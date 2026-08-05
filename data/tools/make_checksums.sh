#!/usr/bin/env bash
# Regenerate checksums.sha256 over every published file. Run from the dataset root
# after adding or changing anything.
set -euo pipefail
cd "$(dirname "$0")/.."
find . -type f ! -name checksums.sha256 ! -name '.gitkeep' -print0 \
  | sort -z | xargs -0 sha256sum > checksums.sha256
echo "wrote checksums.sha256 ($(wc -l < checksums.sha256) files)"
