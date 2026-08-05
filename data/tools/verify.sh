#!/usr/bin/env bash
# Verify a downloaded copy of the dataset.
set -euo pipefail
cd "$(dirname "$0")/.."
sha256sum -c checksums.sha256 --quiet && echo "OK: all files match checksums.sha256"
