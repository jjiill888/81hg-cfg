#!/usr/bin/env bash
# Build 81hg_kbd firmware. Output: ~/桌面/zmk/app/build/81hg/zephyr/zmk.uf2
# Full rebuild: ./build.sh -p
set -e

ZMK=$HOME/桌面/zmk
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=$HOME/zephyr-sdk-0.17.4
export PATH=$ZMK/.venv/bin:$HOME/.local/bin:$PATH

PRISTINE=auto
[ "$1" = "-p" ] && PRISTINE=always

cd "$ZMK/app"
west build -p $PRISTINE -b nrfmicro/nrf52840 -d build/81hg -- \
    -DSHIELD=81hg_kbd \
    -DZMK_CONFIG=$HOME/桌面/81hg-cfg/config

echo
echo "固件: $ZMK/app/build/81hg/zephyr/zmk.uf2"
