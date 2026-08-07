#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps="$root/.deps"
source_dir="$deps/libsurvive-src"
prefix="$deps/libsurvive"
build_dir="$root/build/linux-vive"
libsurvive_build="$root/build/libsurvive"
libsurvive_repo="https://github.com/collabora/libsurvive.git"
libsurvive_commit="f23d298e6f0e69746eae1b7cf847e269d61e35f9"

if [[ ! -x "$root/.venv/bin/python" ]]; then
  echo "Missing $root/.venv; run 'uv sync --locked --no-dev' first." >&2
  exit 1
fi

mkdir -p "$deps" "$root/build"
if [[ ! -d "$source_dir/.git" ]]; then
  git clone "$libsurvive_repo" "$source_dir"
fi
git -C "$source_dir" fetch --depth 1 origin "$libsurvive_commit"
git -C "$source_dir" checkout --detach "$libsurvive_commit"

cmake -S "$source_dir" -B "$libsurvive_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$libsurvive_build" --parallel "$(nproc)"
cmake --install "$libsurvive_build" --prefix "$prefix"

PKG_CONFIG_PATH="$prefix/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}" \
  cmake -S "$root/native" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DUMI_LIBSURVIVE_PREFIX="$prefix"
cmake --build "$build_dir" --parallel "$(nproc)"
cmake --install "$build_dir" --prefix "$root/.venv"

"$root/.venv/bin/umi-vive-linux" --self-test
echo "Installed $root/.venv/bin/umi-vive-linux"
