#!/usr/bin/env bash
# mount_fleet.sh (2026-11-10a, build e12, Nick 10 Nov): mount the USB stick labelled FLEET (ext4) at /media/$USER/FLEET
# if it is not mounted yet. Safe to run again and again: an already-mounted stick is left alone.
# It needs sudo for the mount itself, so it SAYS each thing it does before doing it. It never edits a system file
# (the one-time line for /etc/fstab, so the stick mounts at every start-up, is in docs/how-to/usb-stick.md for Nick).
#
#     tools/pi/mount_fleet.sh            mount it if needed
#     tools/pi/mount_fleet.sh --check    only say whether it is mounted (exit 0 yes, 1 no); never asks for a password
set -u
VERSION="2026-11-10a"
LABEL="${FLEET_LABEL:-FLEET}"
USERNAME="${SUDO_USER:-${USER:-nick}}"
MNT="${FLEET_MOUNT:-/media/$USERNAME/$LABEL}"

where_mounted() {           # the folder the stick is mounted on, or nothing
  findmnt -n -o TARGET --source "LABEL=$LABEL" 2>/dev/null | head -1
}

M="$(where_mounted)"
if [ -n "$M" ]; then
  echo "The USB stick $LABEL is mounted at $M: nothing to do."
  [ "$M" = "$MNT" ] || echo "(Note: the runs look for it under /media/$USERNAME; $M works if it is there too.)"
  exit 0
fi
if [ "${1:-}" = "--check" ]; then
  echo "The USB stick $LABEL is NOT mounted."
  exit 1
fi
if ! blkid -L "$LABEL" >/dev/null 2>&1 && [ ! -e "/dev/disk/by-label/$LABEL" ]; then
  echo "No disk labelled $LABEL is plugged in (checked /dev/disk/by-label). Plug the stick in and run this again."
  exit 1
fi
DEV="$(blkid -L "$LABEL" 2>/dev/null || readlink -f "/dev/disk/by-label/$LABEL")"
echo "Found the stick $LABEL at $DEV."
echo "Making the folder $MNT (sudo mkdir -p $MNT) ..."
sudo mkdir -p "$MNT" || { echo "Could not make $MNT."; exit 1; }
echo "Mounting it there (sudo mount LABEL=$LABEL $MNT) ..."
sudo mount "LABEL=$LABEL" "$MNT" || { echo "The mount failed (see the message above)."; exit 1; }
echo "Letting $USERNAME write to it (sudo chown $USERNAME: $MNT) ..."
sudo chown "$USERNAME:" "$MNT" || echo "(Could not change its owner: frames may not save there.)"
echo "Done: $(df -h "$MNT" | tail -1 | awk '{print $4 " free on " $6}')"
