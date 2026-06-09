# MockSat -- Mock WMS Satellite Server

HTTPS server that mimics GIBS and EUMETView WMS GetMap endpoints for testing
ESP32 firmware sync without hitting real servers.

## Setup

```bash
cd tools/mocksat
./generate_cert.sh
python3 server.py
```

## Point device at mock server

```bash
curl 'http://satwatch.local/seturl?gibs=https://LAPTOP_IP:4443&eumet=https://LAPTOP_IP:4443'
```

## Fault injection

Inject a fault that matches requests containing a URL substring:

```bash
# All EUMETView requests get connection-closed (forever)
curl -X POST http://localhost:4443/ctl \
  -d '{"match":"mtg_fd","fault":"close","times":9999}'

# Next 3 GIBS requests return 503
curl -X POST http://localhost:4443/ctl \
  -d '{"match":"GOES","fault":"503","times":3}'

# Truncate response at 30% of body
curl -X POST http://localhost:4443/ctl \
  -d '{"match":"GOES","fault":"truncate","arg":30,"times":1}'

# Slow drip at 512 bytes/sec
curl -X POST http://localhost:4443/ctl \
  -d '{"match":"GOES","fault":"slow","arg":512,"times":1}'

# Corrupt JPEG scan data
curl -X POST http://localhost:4443/ctl \
  -d '{"match":"GOES","fault":"corrupt","times":1}'

# Serve all-black frame (validator test)
curl -X POST http://localhost:4443/ctl \
  -d '{"match":"GOES","fault":"black","times":1}'
```

### Fault table

| fault      | arg          | effect                                    |
|------------|--------------|-------------------------------------------|
| `ok`       | --           | normal response                           |
| `close`    | --           | `Connection: close` header, no body       |
| `429`      | --           | HTTP 429, empty body                      |
| `503`      | --           | HTTP 503, empty body                      |
| `truncate` | percent 1-99 | send arg% of body then close              |
| `slow`     | bytes/sec    | drip body at given rate                   |
| `corrupt`  | --           | flip bytes after SOS marker               |
| `black`    | --           | serve `fixtures/black.jpg` instead        |

## Control endpoints

```bash
# Reset all fault rules
curl http://localhost:4443/reset

# Show active rules and request count
curl http://localhost:4443/status
```

## Fixtures

Place real JPEG captures in `fixtures/`. See `fixtures/README.txt` for
capture commands. Fixture routing by LAYERS parameter:

- LAYERS contains "GOES" -> `fixtures/gibs_baseline.jpg`
- LAYERS contains "mtg_fd" -> `fixtures/eumet_progressive.jpg`
- Default -> `fixtures/gibs_baseline.jpg`
