#!/usr/bin/env bash
# 00-setup-share.sh — make the UTM "vm-share" folder permanently available
# inside Ubuntu at ~/vm-share with your user as owner.
#
# Run INSIDE the Ubuntu VM, once, after the first manual mount:
#   sudo mkdir -p /mnt/utm
#   sudo mount -t 9p -o trans=virtio,version=9p2000.L share /mnt/utm
#   bash /mnt/utm/scripts/00-setup-share.sh
#
# How it works (UTM VirtFS guide: https://docs.getutm.app/guest-support/linux/):
#   1. The Mac folder is exported by QEMU as a 9p device tagged "share".
#   2. /etc/fstab mounts it at /mnt/utm on every boot.
#   3. Files there carry the Mac's user id (usually 501), not yours (1000),
#      so bindfs re-maps ownership and exposes it at ~/vm-share.
set -euo pipefail

if [[ $EUID -eq 0 ]]; then
  echo "Run this as your normal user (it calls sudo when needed), not with sudo." >&2
  exit 1
fi

ME_UID=$(id -u); ME_GID=$(id -g)
TARGET="$HOME/vm-share"

echo "==> Installing bindfs (ownership re-mapping)"
sudo apt-get update
sudo apt-get install -y bindfs

echo "==> Ensuring /mnt/utm is mounted"
sudo mkdir -p /mnt/utm
if ! mountpoint -q /mnt/utm; then
  sudo mount -t 9p -o trans=virtio,version=9p2000.L share /mnt/utm
fi

HOST_UID=$(stat -c %u /mnt/utm)
HOST_GID=$(stat -c %g /mnt/utm)
echo "    Mac owner of the share: uid=$HOST_UID gid=$HOST_GID -> mapped to you: uid=$ME_UID gid=$ME_GID"

echo "==> Adding /etc/fstab entries (backup: /etc/fstab.bak-vmshare)"
sudo cp -n /etc/fstab /etc/fstab.bak-vmshare || true
LINE1="share /mnt/utm 9p trans=virtio,version=9p2000.L,rw,_netdev,nofail,auto 0 0"
LINE2="/mnt/utm $TARGET fuse.bindfs map=${HOST_UID}/${ME_UID}:@${HOST_GID}/@${ME_GID},x-systemd.requires=/mnt/utm,_netdev,nofail,auto 0 0"
grep -qF "share /mnt/utm 9p" /etc/fstab || echo "$LINE1" | sudo tee -a /etc/fstab >/dev/null
grep -qF "/mnt/utm $TARGET fuse.bindfs" /etc/fstab || echo "$LINE2" | sudo tee -a /etc/fstab >/dev/null

echo "==> Mounting ~/vm-share"
mkdir -p "$TARGET"
sudo systemctl daemon-reload
mountpoint -q "$TARGET" || sudo mount "$TARGET"

echo "==> Write test"
TEST="$TARGET/.vm-write-test"
if echo ok > "$TEST" && rm -f "$TEST"; then
  echo "OK: $TARGET is mounted and writable. Files here are the same as vm-share on your Mac."
else
  echo "WARNING: $TARGET is not writable. Check 'ls -na /mnt/utm'." >&2
  exit 1
fi
ls -la "$TARGET"
