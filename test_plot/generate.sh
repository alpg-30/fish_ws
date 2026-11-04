#!/usr/bin/env bash
# 生成 Task003 波形核对图：C++ 真实实现导出 CSV -> 纯 Python 渲染 SVG。
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src="$here/../src/fish_control"
build="$here/.build"
output="$here/output"

mkdir -p "$build" "$output"

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -I"$src/include" \
  "$here/dump_waveform.cpp" "$src/src/config.cpp" "$src/src/fluent_waveform.cpp" \
  -o "$build/dump_waveform"

"$build/dump_waveform" "$output"
python3 "$here/render_svg.py" "$output"
