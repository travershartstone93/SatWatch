#!/usr/bin/env python3
"""Mock WMS satellite server for LiveSat integration testing.

Serves JPEG fixtures over HTTPS, matching GIBS and EUMETView URL patterns.
Supports fault injection via POST /ctl and GET /reset.
"""

import json
import os
import random
import socket
import ssl
import struct
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

FIXTURES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")
DEFAULT_PORT = 4443

# Global fault rules: list of {"match": str, "fault": str, "arg": num, "remaining": int}
_rules_lock = threading.Lock()
_rules: list[dict] = []


def find_fixture(layers: str, url: str) -> str:
    """Resolve a fixture file path from LAYERS param or URL pattern."""
    # Try exact match on LAYERS value
    if layers:
        candidate = os.path.join(FIXTURES_DIR, layers.replace(":", "_") + ".jpg")
        if os.path.isfile(candidate):
            return candidate

    # Fallback: first .jpg in fixtures/
    for f in sorted(os.listdir(FIXTURES_DIR)):
        if f.endswith(".jpg"):
            return os.path.join(FIXTURES_DIR, f)

    return ""


def match_rule(url: str) -> dict | None:
    """Find and consume the first matching fault rule."""
    with _rules_lock:
        for rule in _rules:
            if rule["match"] in url:
                if rule["remaining"] > 0:
                    rule["remaining"] -= 1
                    if rule["remaining"] == 0:
                        _rules.remove(rule)
                elif rule["remaining"] == -1:
                    pass  # infinite
                return rule
    return None


def is_eumetview(url: str, layers: str) -> bool:
    return "view.eumetsat.int" in url or "mtg_fd" in layers


class MockHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        print(f"[mocksat] {self.address_string()} {fmt % args}")

    def do_GET(self):
        if self.path == "/reset":
            with _rules_lock:
                _rules.clear()
            self._respond(200, b"ok", "text/plain")
            return

        # Parse query
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        layers = qs.get("LAYERS", qs.get("layers", [""]))[0]
        full_url = self.path

        # Check fault rules
        rule = match_rule(full_url)
        fault = rule["fault"] if rule else "ok"
        arg = rule.get("arg", 0) if rule else 0

        # Status-code faults
        if fault == "429":
            self._respond(429, b"", "text/plain")
            return
        if fault == "503":
            self._respond(503, b"", "text/plain")
            return

        # Resolve fixture
        if fault == "black":
            fpath = os.path.join(FIXTURES_DIR, "black.jpg")
        else:
            fpath = find_fixture(layers, full_url)

        if not fpath or not os.path.isfile(fpath):
            self._respond(404, b"no fixture found", "text/plain")
            return

        with open(fpath, "rb") as f:
            body = f.read()

        # Apply body-level faults
        if fault == "corrupt" and len(body) > 200:
            body = bytearray(body)
            for _ in range(random.randint(5, 20)):
                pos = random.randint(100, len(body) - 1)
                body[pos] = body[pos] ^ random.randint(1, 255)
            body = bytes(body)

        if fault == "truncate":
            frac = max(0.0, min(1.0, float(arg)))
            body = body[: int(len(body) * frac)]

        eumet = is_eumetview(full_url, layers)

        # Send headers
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")

        if fault == "close":
            self.send_header("Connection", "close")

        if eumet:
            self.send_header("Transfer-Encoding", "chunked")
        else:
            self.send_header("Content-Length", str(len(body)))

        self.end_headers()

        # Send body
        try:
            if fault == "slow":
                bps = max(1, int(arg))
                sent = 0
                while sent < len(body):
                    chunk = body[sent : sent + bps]
                    if eumet:
                        self._write_chunk(chunk)
                    else:
                        self.wfile.write(chunk)
                    self.wfile.flush()
                    sent += len(chunk)
                    time.sleep(1.0)
                if eumet:
                    self._write_chunk(b"")  # terminal chunk
            elif eumet:
                # Chunked encoding
                chunk_size = 4096
                for i in range(0, len(body), chunk_size):
                    self._write_chunk(body[i : i + chunk_size])
                self._write_chunk(b"")  # terminal chunk
            else:
                self.wfile.write(body)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        if self.path != "/ctl":
            self._respond(404, b"not found", "text/plain")
            return

        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length)
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError:
            self._respond(400, b"bad json", "text/plain")
            return

        match_str = obj.get("match", "")
        fault = obj.get("fault", "ok")
        arg = obj.get("arg", 0)
        times = obj.get("times", -1)  # -1 = infinite

        with _rules_lock:
            _rules.append({
                "match": match_str,
                "fault": fault,
                "arg": arg,
                "remaining": times,
            })

        self._respond(200, json.dumps({"status": "added"}).encode(), "application/json")

    def _respond(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _write_chunk(self, data: bytes):
        self.wfile.write(f"{len(data):x}\r\n".encode())
        self.wfile.write(data)
        self.wfile.write(b"\r\n")
        self.wfile.flush()


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Mock satellite WMS server")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--cert", default=os.path.join(os.path.dirname(__file__), "mock.crt"))
    parser.add_argument("--key", default=os.path.join(os.path.dirname(__file__), "mock.key"))
    parser.add_argument("--no-tls", action="store_true", help="Run plain HTTP (no TLS)")
    args = parser.parse_args()

    server = HTTPServer(("0.0.0.0", args.port), MockHandler)

    if not args.no_tls:
        if not os.path.isfile(args.cert) or not os.path.isfile(args.key):
            print(f"Certificate not found. Run gen_cert.sh first:")
            print(f"  cd {os.path.dirname(__file__)} && bash gen_cert.sh")
            raise SystemExit(1)
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(args.cert, args.key)
        server.socket = ctx.wrap_socket(server.socket, server_side=True)
        proto = "https"
    else:
        proto = "http"

    print(f"[mocksat] Listening on {proto}://0.0.0.0:{args.port}")
    print(f"[mocksat] Fixtures dir: {FIXTURES_DIR}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[mocksat] Shutting down.")
        server.shutdown()


if __name__ == "__main__":
    main()
