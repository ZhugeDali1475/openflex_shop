#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime="${root}/local_dependencies/roboos_runtime/redis"
export LD_LIBRARY_PATH="${runtime}/root/usr/lib/x86_64-linux-gnu"
exec "${runtime}/root/usr/bin/redis-server" --bind 127.0.0.1 --port 16383 \
    --protected-mode yes --save '' --appendonly no --dir "${runtime}"
