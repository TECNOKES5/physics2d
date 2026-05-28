#!/usr/bin/env bash
# ─── physics2d build script (Arch Linux) ──────────────────────────────────────
set -e

CXX="${CXX:-g++}"
OUT="physics2d"
SRC="src/main.cpp"
FLAGS="-std=c++17 -O2 -Wall -Wextra"
LIBS="-lGL -lglfw"

echo "▶  Checking dependencies..."
if ! pacman -Qq glfw-x11 &>/dev/null && ! pacman -Qq glfw-wayland &>/dev/null; then
    echo "   GLFW not found. Install with:"
    echo "     sudo pacman -S glfw-x11     (for X11)"
    echo "     sudo pacman -S glfw-wayland (for Wayland)"
    exit 1
fi
echo "   GLFW OK"

echo "▶  Compiling..."
$CXX $FLAGS $SRC -o $OUT $LIBS -I src

echo "✔  Built: ./$OUT"
echo ""
echo "Controls:"
echo "  Left click     → spawn body at cursor"
echo "  C              → spawn circles"
echo "  B              → spawn boxes"
echo "  R              → reset scene"
echo "  G              → flip gravity"
echo "  Space          → pause / unpause"
echo "  Escape         → quit"
echo ""
echo "▶  Running..."
./$OUT
