#!/bin/bash
set -e

# W5-2 split-edge materialization is a CLI-only Rust mission.
# No extra services are required; just verify the workspace and toolchain.

cargo --version >/dev/null
rustc --version >/dev/null
cargo metadata --no-deps --format-version 1 >/dev/null

echo "truck-bop environment ready (cargo CLI surface)"
