#!/usr/bin/env bash
# Runs ON THE RASPBERRY with sudo:
#   ssh -t jarvis@<ip> 'sudo bash ~/src/ak7375/reinstalar_ak7375.sh'
#
# The IMX519 camera's focus motor (AK7375) uses a driver compiled by hand,
# which is only installed for the kernel it was compiled for. A kernel update
# leaves the camera without it, and libcamera then reports "No cameras available!".
# This script recompiles the driver for the running kernel, holds the kernel
# packages so an automatic update cannot break it again, and reboots.
#
# To allow kernel updates again: sudo apt-mark unhold linux-raspi linux-image-raspi
# (then run this script again after rebooting into the new kernel).

set -euo pipefail

KVER="${1:-$(uname -r)}"
DIR=/home/jarvis/src/ak7375

[[ $EUID -eq 0 ]] || { echo "Ejecútalo con sudo."; exit 1; }
[[ -f "$DIR/ak7375.c" ]] || { echo "No existe $DIR/ak7375.c"; exit 1; }

echo "== Instalando los encabezados del kernel $KVER"
apt-get update
apt-get install -y build-essential "linux-headers-$KVER"

echo "== Compilando el driver de enfoque AK7375"
sudo -u jarvis make -C "$DIR" KVER="$KVER" clean all

echo "== Instalando el driver"
install -D -m 644 "$DIR/ak7375.ko" "/lib/modules/$KVER/updates/ak7375.ko"
depmod -a "$KVER"
echo ak7375 > /etc/modules-load.d/ak7375.conf
echo "vermagic: $(modinfo -F vermagic "/lib/modules/$KVER/updates/ak7375.ko")"

echo "== Bloqueando las actualizaciones automáticas del kernel"
held=()
for pkg in linux-raspi linux-image-raspi linux-headers-raspi linux-modules-raspi; do
    if dpkg-query -W -f='${db:Status-Status}' "$pkg" 2>/dev/null | grep -qx installed; then
        held+=("$pkg")
    fi
done
(( ${#held[@]} )) && apt-mark hold "${held[@]}"

echo
echo "LISTO. Reiniciando para cargar el driver..."
reboot
