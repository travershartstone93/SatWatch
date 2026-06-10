#!/bin/bash
set -euo pipefail

FQBN="esp32:esp32:waveshare_esp32_s3_touch_amoled_206:PSRAM=enabled,PartitionScheme=custom"
BUILD_PATH="/tmp/s3-build"
SKETCH="esp32-LiveSat-s3"
OTA_REPO="travershartstone93/LiveSat-OTA"
BIN="$BUILD_PATH/$SKETCH.ino.bin"

if [ $# -lt 1 ]; then
  echo "Usage: $0 <version_number>"
  echo "  Compiles firmware and publishes a GitHub release."
  exit 1
fi

VERSION="$1"
TAG="v$VERSION"

echo "=== Compiling firmware (version $VERSION) ==="
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_PATH" "$SKETCH"

if [ ! -f "$BIN" ]; then
  echo "ERROR: Binary not found at $BIN"
  exit 1
fi

SIZE=$(stat -c%s "$BIN")
echo "=== Binary size: $SIZE bytes ==="

MD5=$(md5sum "$BIN" | cut -d' ' -f1)
echo '{"version": '"$VERSION"', "size": '"$SIZE"', "md5": "'"$MD5"'"}' > /tmp/version.json
echo "=== MD5: $MD5 ==="
cp "$BIN" /tmp/firmware.bin

echo "=== Creating GitHub release $TAG ==="
# Delete existing release/tag if re-deploying same version
gh release delete "$TAG" --repo "$OTA_REPO" --yes 2>/dev/null || true
gh release create "$TAG" \
  --repo "$OTA_REPO" \
  --title "Firmware $TAG" \
  --notes "Version $VERSION ($SIZE bytes)" \
  /tmp/firmware.bin \
  /tmp/version.json

echo "=== Deployed version $VERSION ($SIZE bytes) to $OTA_REPO ==="
