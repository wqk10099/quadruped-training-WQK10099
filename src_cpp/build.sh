#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 默认同时编译单线程和双线程两个目标
DEFAULT_TARGETS=(
    multimode_main
    multimode_main_threaded
)

if (( $# > 0 )); then
    TARGETS=("$@")
else
    TARGETS=("${DEFAULT_TARGETS[@]}")
fi

CXX="${CXX:-g++}"
CXXFLAGS="${CXXFLAGS:--std=c++17 -Wall -Wextra -O2}"

read -r -a CXXFLAGS_ARRAY <<< "$CXXFLAGS"

command -v "$CXX" >/dev/null 2>&1 || {
    echo "error: C++ compiler not found: $CXX" >&2
    exit 1
}

command -v python3 >/dev/null 2>&1 || {
    echo "error: python3 not found" >&2
    exit 1
}

command -v pkg-config >/dev/null 2>&1 || {
    echo "error: pkg-config not found" >&2
    exit 1
}

pkg-config --exists sdl2 || {
    echo "error: SDL2 development package not found" >&2
    exit 1
}

MUJOCO_DIR="$(
    python3 -c 'import mujoco, pathlib; print(pathlib.Path(mujoco.__file__).resolve().parent)'
)"

MUJOCO_INCLUDE="$MUJOCO_DIR/include"

shopt -s nullglob
MUJOCO_LIBS=("$MUJOCO_DIR"/libmujoco.so.*)
shopt -u nullglob

if (( ${#MUJOCO_LIBS[@]} == 0 )); then
    echo "error: libmujoco.so.* not found in $MUJOCO_DIR" >&2
    exit 1
fi

MUJOCO_LIB="$(
    printf '%s\n' "${MUJOCO_LIBS[@]}" | sort -V | tail -n 1
)"

mapfile -t SDL_CFLAGS < <(pkg-config --cflags sdl2)
mapfile -t SDL_LIBS < <(pkg-config --libs sdl2)

build_target() {
    local target="$1"
    local source_file="$SCRIPT_DIR/${target}.cpp"
    local output_file="$SCRIPT_DIR/${target}"

    if [[ ! -f "$source_file" ]]; then
        echo "error: source file not found: $source_file" >&2
        return 1
    fi

    echo
    echo "building: $target"
    echo "source  : $source_file"
    echo "output  : $output_file"

    "$CXX" \
        "${CXXFLAGS_ARRAY[@]}" \
        "${SDL_CFLAGS[@]}" \
        -pthread \
        -I"$MUJOCO_INCLUDE" \
        "$SCRIPT_DIR/simulator.cpp" \
        "$SCRIPT_DIR/controller.cpp" \
        "$source_file" \
        "$MUJOCO_LIB" \
        -Wl,-rpath,"$MUJOCO_DIR" \
        "${SDL_LIBS[@]}" \
        -lGL \
        -o "$output_file"

    echo "built successfully: $output_file"
}

for target in "${TARGETS[@]}"; do
    build_target "$target"
done