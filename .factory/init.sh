#!/usr/bin/env python3
import subprocess


def run(command: list[str]) -> None:
    subprocess.run(command, check=True)


run(["rustc", "--version"])
run(["cargo", "--version"])
run(["cargo", "metadata", "--no-deps"])

print("truck-bop sphere-box mission environment ready (no external services required)")
