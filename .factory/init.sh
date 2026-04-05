#!/bin/bash
set -e

# No special services are required for truck-bop.
# Verify the Rust workspace is readable before workers start.

cargo --version >/dev/null
rustc --version >/dev/null
cargo metadata --no-deps >/dev/null

echo "truck-bop environment ready"
