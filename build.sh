#!/bin/bash
# Script de build ZLinky firmware JN5189
# Usage: ./build.sh [clean]

MAKE="/c/nxp/MCUXpressoIDE_11.2.0_4120/ide/plugins/com.nxp.mcuxpresso.tools.win32_11.2.0.202001021529/buildtools/bin/make.exe"
BUILD_DIR="C:/Users/akila/Desktop/lixee/workspace/JN-AN-1246/Zlinky_TIC_sx1280/ZLinky/Build/mcux"
BIN_DIR="C:/Users/akila/Desktop/lixee/workspace/JN-AN-1246/Zlinky_TIC_sx1280/Binaries/ZLinky_GpProxy"

cd "$BUILD_DIR" || exit 1

if [ "$1" = "clean" ]; then
    echo "=== CLEAN ==="
    "$MAKE" clean -r 2>&1 | tail -3
    echo ""
fi

echo "=== BUILD ==="
"$MAKE" all -r 2>&1 | grep -E "error:|warning:|Linking|text|\.bin|Size"

if [ $? -eq 0 ]; then
    echo ""
    echo "=== BUILD OK ==="
    ls -la "$BIN_DIR/ZLinky_GpProxy.bin" 2>/dev/null
else
    echo ""
    echo "=== BUILD FAILED ==="
    exit 1
fi
