#!/bin/bash
set -euo pipefail

rustc --version
cargo --version
cargo metadata --no-deps >/dev/null

echo "truck-bop sphere-box mission environment ready (no external services required)"
