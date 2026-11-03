#!/usr/bin/env bash
# 从项目根目录启动；只使用已经存在的 conda 环境，不安装或下载依赖。
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ "${CONDA_DEFAULT_ENV:-}" == "dexonomy-isaac" ]]; then
    exec python scripts/run_sim.py "$@"
fi
# 优先使用当前 conda 配置，再查找常见安装位置，避免写死本机用户名。
conda_executable="${CONDA_EXE:-}"
if [[ ! -x "$conda_executable" ]]; then
    conda_executable="$(command -v conda || true)"
fi
if [[ ! -x "$conda_executable" ]]; then
    for candidate in "$HOME/anaconda3/bin/conda" "$HOME/miniconda3/bin/conda" "$HOME/miniforge3/bin/conda"; do
        if [[ -x "$candidate" ]]; then
            conda_executable="$candidate"
            break
        fi
    done
fi
if [[ ! -x "$conda_executable" ]]; then
    echo "未找到 conda。请先激活 dexonomy-isaac 环境，再运行此脚本。" >&2
    exit 1
fi
exec "$conda_executable" run --no-capture-output -n dexonomy-isaac python scripts/run_sim.py "$@"
