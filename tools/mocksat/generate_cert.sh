#!/bin/bash
cd "$(dirname "$0")"
openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -keyout mock.key -out mock.crt -subj "/CN=mocksat"
echo "Generated mock.crt and mock.key"
