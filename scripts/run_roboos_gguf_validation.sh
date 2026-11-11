#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_path="$project_root/.venvs/roboos-client/bin/python"
if [[ ! -x "$python_path" ]]; then
    printf '缺少独立客户端环境，请参阅 ROBOOS_GGUF_INTEGRATION_CN.md。\n' >&2
    exit 1
fi
cd "$project_root"
exec env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY \
    -u http_proxy -u https_proxy -u all_proxy \
    "$python_path" -I "$project_root/scripts/validate_roboos_gguf_integration.py" "$@"
