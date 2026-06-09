# mocksat -- Mock Satellite WMS Server

HTTPS mock for GIBS and EUMETView WMS endpoints. Serves JPEG fixtures with fault injection for testing firmware resilience.

## Setup

```bash
cd tools/mocksat
bash gen_cert.sh        # one-time: creates mock.crt / mock.key
```

## Add fixtures

Drop JPEG files into `fixtures/`. Name them to match the LAYERS parameter with colons replaced by underscores:

```
fixtures/GOES-East_ABI_GeoColor.jpg
fixtures/mtg_fd_rgb_geocolour.jpg
fixtures/black.jpg                    # used by the "black" fault
```

If no exact match is found, the server serves the first `.jpg` alphabetically.

## Start

```bash
python3 server.py                     # https://0.0.0.0:4443
python3 server.py --port 8443         # custom port
python3 server.py --no-tls            # plain HTTP (for quick tests)
```

## Fault injection

Inject a rule (applies to any URL containing the match substring):

```bash
# Return 503 for the next 3 GIBS requests
curl -k -X POST https://localhost:4443/ctl \
  -d '{"match": "GeoColor", "fault": "503", "times": 3}'

# Truncate EUMETView responses to 30% forever
curl -k -X POST https://localhost:4443/ctl \
  -d '{"match": "mtg_fd", "fault": "truncate", "arg": 0.3, "times": -1}'

# Drip at 500 bytes/sec
curl -k -X POST https://localhost:4443/ctl \
  -d '{"match": "GeoColor", "fault": "slow", "arg": 500, "times": 1}'
```

Clear all rules:

```bash
curl -k https://localhost:4443/reset
```

### Fault table

| fault      | arg                | effect                                 |
|------------|--------------------|----------------------------------------|
| `ok`       | --                 | normal response                        |
| `close`    | --                 | adds `Connection: close` header        |
| `429`      | --                 | HTTP 429, empty body                   |
| `503`      | --                 | HTTP 503, empty body                   |
| `truncate` | 0.0-1.0            | send fraction of body then close       |
| `slow`     | bytes/sec          | drip body at given rate                |
| `corrupt`  | --                 | flip random bytes after byte 100       |
| `black`    | --                 | serve `fixtures/black.jpg` instead     |

## Point firmware at it

In config-s3.h, override the WMS host/port to point at your dev machine running mocksat. The ESP32 must trust the self-signed cert (use `setInsecure()` during testing).
