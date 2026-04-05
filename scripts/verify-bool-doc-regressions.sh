#!/usr/bin/env bash
set -euo pipefail

DOC_FILE="${1:-docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md}"
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

if [[ -f "$DOC_FILE" ]]; then
  DOC_PATH="$DOC_FILE"
elif [[ -f "$ROOT_DIR/$DOC_FILE" ]]; then
  DOC_PATH="$ROOT_DIR/$DOC_FILE"
else
  echo "[error] doc file not found: $DOC_FILE"
  exit 1
fi

python3 - "$DOC_PATH" "$ROOT_DIR" <<'PY'
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path

doc_path = Path(sys.argv[1])
root_dir = Path(sys.argv[2])

# 1) 从文档提取回归命令（仅 markdown 代码块内的行）
commands = []
in_code = False
for raw in doc_path.read_text(encoding="utf-8").splitlines():
    line = raw.strip()
    if line.startswith("```"):
        in_code = not in_code
        continue
    if not in_code:
        continue
    if "cargo test -p truck-bop" in line:
        line = re.sub(r"^[-*]\s*", "", line)
        line = line.strip().strip("`")
        if line:
            commands.append(line)

if not commands:
    print(f"[warn] no cargo test -p truck-bop commands found in {doc_path}")
    sys.exit(0)

print(f"[info] extracted {len(commands)} candidate commands from {doc_path}")

# 2) 获取当前可见测试名
result = subprocess.run(
    ["cargo", "test", "-p", "truck-bop", "--", "--list"],
    cwd=str(root_dir),
    text=True,
    capture_output=True,
    check=True,
)

tests = []
for raw in result.stdout.splitlines():
    m = re.match(r"^(.*?)\s*:\s*test$", raw)
    if m:
        tests.append(m.group(1))

if not tests:
    print("[error] cannot parse any test names from cargo test --list")
    sys.exit(1)


def extract_filter(cmd: str) -> str:
    """Extract cargo test filter argument (近似 cargo test 的 filter 语义）。"""
    if cmd.count("?"):
        raise ValueError("placeholder_detected")

    try:
        tokens = shlex.split(cmd)
    except Exception:
        raise ValueError("unparseable")

    if len(tokens) < 2 or tokens[0] != "cargo" or tokens[1] != "test":
        raise ValueError("not_a_cargo_test")

    parts = tokens[2:]
    if "--" in parts:
        split_at = parts.index("--")
        cargo_parts = parts[:split_at]
        runner_parts = parts[split_at + 1 :]
    else:
        cargo_parts = parts
        runner_parts = []

    idx = 0
    filter_value = ""
    while idx < len(cargo_parts):
        tok = cargo_parts[idx]
        if tok in {"-p", "--package"}:
            idx += 2
            continue
        if tok.startswith("-p") and tok != "-p":
            idx += 1
            continue
        if tok.startswith("--package="):
            idx += 1
            continue
        if tok.startswith("-"):
            idx += 1
            continue
        filter_value = tok
        break
        # noqa: continue
    if not filter_value:
        for tok in runner_parts:
            if tok.startswith("-"):
                continue
            filter_value = tok
            break

    return filter_value

ok = 0
missing = 0
skipped = 0

for cmd in commands:
    try:
        filter_value = extract_filter(cmd)
    except ValueError:
        print(f"[skip] {cmd}")
        skipped += 1
        continue

    if not filter_value:
        print(f"[ok] full-suite check passed: {cmd}")
        ok += 1
        continue

    if any(filter_value in name for name in tests):
        print(f"[ok] filter exists: {filter_value} -> from: {cmd}")
        ok += 1
    else:
        print(f"[missing] no test match for filter '{filter_value}' in command: {cmd}")
        missing += 1

print(f"\n[summary] commands: ok={ok}, missing={missing}, skipped={skipped}")

if missing:
    sys.exit(1)
PY