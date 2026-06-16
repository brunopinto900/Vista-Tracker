#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$SCRIPT_DIR/.."
BUILD="$ROOT/cpp/build"

usage() {
    echo "Usage: $0 [scenario]"
    echo ""
    echo "  $0                 run default config"
    echo "  $0 go_around       run a named scenario"
    echo "  $0 --list          list available scenarios"
    echo ""
    echo "Available scenarios:"
    ls "$ROOT/config/scenarios/"*.yaml 2>/dev/null \
        | xargs -I{} basename {} .yaml \
        | sed 's/^/  /'
}

if [[ "${1}" == "-h" || "${1}" == "--help" ]]; then
    usage
    exit 0
fi

# ── Resolve config path (absolute) ────────────────────────────────────────────
if [[ "${1}" == "--list" || -z "${1}" ]]; then
    CONFIG_ABS="$(realpath "$ROOT/config/config.yaml")"
elif [[ -f "${1}" ]]; then
    CONFIG_ABS="$(realpath "${1}")"
elif [[ -f "$ROOT/config/scenarios/${1}.yaml" ]]; then
    CONFIG_ABS="$(realpath "$ROOT/config/scenarios/${1}.yaml")"
else
    echo "error: scenario '${1}' not found" >&2
    exit 1
fi

# ── Build ─────────────────────────────────────────────────────────────────────
echo "[run] building tracker..."
mkdir -p "$BUILD"
cmake -S "$ROOT/cpp" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=OFF > /dev/null
make -C "$BUILD" -j"$(nproc)" --no-print-directory

# ── Simulate ──────────────────────────────────────────────────────────────────
cd "$BUILD"

if [[ "${1}" == "--list" ]]; then
    ./tracker --list
    exit 0
elif [[ -n "${1}" ]]; then
    echo "[run] scenario: ${1}"
    ./tracker "${1}"
else
    echo "[run] scenario: default"
    ./tracker
fi

# ── Visualise ─────────────────────────────────────────────────────────────────
echo "[run] launching visualizer..."
cd "$ROOT/python"
python3 visualize.py "$CONFIG_ABS"
