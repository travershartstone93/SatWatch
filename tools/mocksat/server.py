#!/usr/bin/env python3
"""Mock WMS satellite server for testing ESP32 firmware sync.

Mimics GIBS and EUMETView WMS GetMap endpoints over HTTPS with fault injection.
"""

import http.server
import json
import os
import ssl
import sys
import socket
import threading
import time
from urllib.parse import parse_qs, urlparse

FIXTURES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")
LISTEN_ADDR = "0.0.0.0"
LISTEN_PORT = 4443

# --- Fault injection state ---
_lock = threading.Lock()
_rules = []        # list of {"match": str, "fault": str, "arg": num, "times": int}
_request_count = 0


def _find_rule(url):
    """Find and consume a matching fault rule for the given URL."""
    with _lock:
        for rule in _rules:
            if rule["match"] in url:
                if rule["times"] > 0:
                    rule["times"] -= 1
                    return dict(rule)
                elif rule["times"] == -1:  # infinite
                    return dict(rule)
        return None


def _log(msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


class MockWMSHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass  # suppress default logging

    def do_POST(self):
        if self.path == "/ctl":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length)
                spec = json.loads(body)
                rule = {
                    "match": spec.get("match", ""),
                    "fault": spec.get("fault", "ok"),
                    "arg": spec.get("arg", 0),
                    "times": spec.get("times", 1),
                }
                with _lock:
                    _rules.append(rule)
                _log(f"CTL added rule: {rule}")
                self._json_response(200, {"ok": True, "rule": rule})
            except Exception as e:
                self._json_response(400, {"error": str(e)})
            return
        self._json_response(404, {"error": "not found"})

    def do_GET(self):
        global _request_count
        with _lock:
            _request_count += 1

        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        _log(f"GET {self.path[:120]}")

        # --- Control endpoints ---
        if path == "/reset":
            with _lock:
                _rules.clear()
            _log("RESET all fault rules")
            self._json_response(200, {"ok": True})
            return

        if path == "/status":
            with _lock:
                snapshot = [dict(r) for r in _rules]
                count = _request_count
            self._json_response(200, {"rules": snapshot, "requests": count})
            return

        # --- WMS GetMap handling ---
        layers = qs.get("LAYERS", qs.get("layers", [""]))[0]
        fixture = self._resolve_fixture(layers)
        if not os.path.isfile(fixture):
            _log(f"FIXTURE MISSING: {fixture}")
            self._json_response(404, {"error": f"fixture not found: {os.path.basename(fixture)}"})
            return

        rule = _find_rule(self.path)
        fault = rule["fault"] if rule else "ok"
        arg = rule["arg"] if rule else 0

        try:
            self._serve_with_fault(fixture, fault, arg)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            _log("Client disconnected")
        except Exception as e:
            _log(f"Error serving: {e}")

    def _resolve_fixture(self, layers):
        layers_lower = layers.lower()
        if "mtg_fd" in layers_lower:
            return os.path.join(FIXTURES_DIR, "eumet_progressive.jpg")
        if "goes" in layers_lower:
            return os.path.join(FIXTURES_DIR, "gibs_baseline.jpg")
        return os.path.join(FIXTURES_DIR, "gibs_baseline.jpg")

    def _serve_with_fault(self, filepath, fault, arg):
        data = open(filepath, "rb").read()

        if fault == "429":
            self.send_response(429)
            self.send_header("Retry-After", "60")
            self.send_header("Content-Length", "0")
            self.end_headers()
            _log("FAULT 429")
            return

        if fault == "503":
            self.send_response(503)
            self.send_header("Content-Length", "0")
            self.end_headers()
            _log("FAULT 503")
            return

        if fault == "close":
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Connection", "close")
            self.end_headers()
            _log("FAULT close (no body)")
            return

        if fault == "black":
            black_path = os.path.join(FIXTURES_DIR, "black.jpg")
            if os.path.isfile(black_path):
                data = open(black_path, "rb").read()
                _log("FAULT black")
            else:
                _log("FAULT black (missing black.jpg, serving original)")

        if fault == "corrupt":
            data = self._corrupt_after_sos(data)
            _log("FAULT corrupt")

        if fault == "truncate":
            pct = max(1, min(99, int(arg))) if arg else 50
            orig_len = len(data)
            cut = orig_len * pct // 100
            data = data[:cut]
            _log(f"FAULT truncate {pct}% ({cut}/{orig_len} bytes)")

        # Send response with chunked transfer encoding (like EUMETView)
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()

        if fault == "slow":
            bps = max(64, int(arg)) if arg else 1024
            _log(f"FAULT slow drip at {bps} bytes/sec")
            self._send_chunked_slow(data, bps)
        else:
            self._send_chunked(data)

    def _send_chunked(self, data, chunk_size=8192):
        offset = 0
        while offset < len(data):
            end = min(offset + chunk_size, len(data))
            chunk = data[offset:end]
            self.wfile.write(f"{len(chunk):X}\r\n".encode())
            self.wfile.write(chunk)
            self.wfile.write(b"\r\n")
            offset = end
        self.wfile.write(b"0\r\n\r\n")
        self.wfile.flush()

    def _send_chunked_slow(self, data, bps):
        interval = 0.1
        bytes_per_tick = max(1, int(bps * interval))
        offset = 0
        while offset < len(data):
            end = min(offset + bytes_per_tick, len(data))
            chunk = data[offset:end]
            self.wfile.write(f"{len(chunk):X}\r\n".encode())
            self.wfile.write(chunk)
            self.wfile.write(b"\r\n")
            self.wfile.flush()
            offset = end
            time.sleep(interval)
        self.wfile.write(b"0\r\n\r\n")
        self.wfile.flush()

    def _corrupt_after_sos(self, data):
        """Flip bytes after the SOS (0xFFDA) marker."""
        sos = data.find(b"\xff\xda")
        if sos == -1:
            return data
        out = bytearray(data)
        for i in range(sos + 12, len(out), 37):
            out[i] ^= 0xFF
        return bytes(out)

    def _json_response(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class ThreadedHTTPServer(http.server.HTTPServer):
    """Handle each request in a new thread (robust against slow faults)."""
    allow_reuse_address = True
    daemon_threads = True

    def process_request(self, request, client_address):
        t = threading.Thread(target=self._handle, args=(request, client_address))
        t.daemon = True
        t.start()

    def _handle(self, request, client_address):
        try:
            self.finish_request(request, client_address)
        except Exception:
            self.handle_error(request, client_address)
        finally:
            self.shutdown_request(request)


def main():
    cert = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mock.crt")
    key = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mock.key")

    if not os.path.isfile(cert) or not os.path.isfile(key):
        print("ERROR: mock.crt / mock.key not found. Run ./generate_cert.sh first.",
              file=sys.stderr)
        sys.exit(1)

    server = ThreadedHTTPServer((LISTEN_ADDR, LISTEN_PORT), MockWMSHandler)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)

    hostname = socket.gethostname()
    print(f"MockSat HTTPS server on https://{LISTEN_ADDR}:{LISTEN_PORT}")
    print(f"  Host: {hostname}")
    print(f"  Fixtures: {FIXTURES_DIR}")
    print(f"  Control:  POST /ctl | GET /reset | GET /status")
    print()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        server.shutdown()


if __name__ == "__main__":
    main()
