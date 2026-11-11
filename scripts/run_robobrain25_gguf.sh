#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_root="$project_root/local_dependencies/robobrain25_gguf"
binary_dir="$runtime_root/runtime/llama-b12110"
compat_dir="$runtime_root/compat/usr/lib/x86_64-linux-gnu"
loader="$compat_dir/ld-linux-x86-64.so.2"
model_dir="${ROBOBRAIN_MODEL_DIR:-$project_root/local_dependencies/models/RoboBrain2.5-4B-GGUF}"
model_file="$model_dir/RoboBrain2.5-4B.Q4_K_M.gguf"
projector_file="$model_dir/RoboBrain2.5-4B.mmproj-f16.gguf"

for required_file in "$loader" "$binary_dir/llama-server" "$model_file" "$projector_file"; do
    if [[ ! -f "$required_file" ]]; then
        printf '缺少文件：%s\n请参阅 ROBOBRAIN25_GGUF_LOCAL_CN.md。\n' "$required_file" >&2
        exit 1
    fi
done

library_path="$compat_dir:$binary_dir"
for library_dir in "$runtime_root"/runtime/*; do
    if [[ -d "$library_dir" ]]; then
        library_path="$library_path:$library_dir"
    fi
done

cd "$binary_dir"

if [[ "${1:-}" == "--version" || "${1:-}" == "--list-devices" || "${1:-}" == "--help" ]]; then
    exec "$loader" --library-path "$library_path" "$binary_dir/llama-server" "$@"
fi

exec 9>"$runtime_root/runtime.lock"
if ! flock --nonblock 9; then
    printf 'GGUF 服务或安装程序已在运行，请先结束已有进程。\n' >&2
    exit 1
fi

exec "$loader" --library-path "$library_path" "$binary_dir/llama-server" \
    --model "$model_file" \
    --mmproj "$projector_file" \
    --alias robobrain2.5-4b-gguf \
    --host 127.0.0.1 \
    --cors-origins localhost \
    --port "${ROBOBRAIN_PORT:-4567}" \
    --ctx-size 4096 \
    --parallel 1 \
    --n-gpu-layers 99 \
    --device CUDA0 \
    --fit off \
    --flash-attn on \
    --batch-size 512 \
    --ubatch-size 128 \
    --image-min-tokens 1024 \
    --image-max-tokens 1024 \
    --n-predict 512 \
    --jinja \
    --reasoning off \
    "$@"
