#!/usr/bin/env bash
# One-time install of the start page (tools/pi/launcher.py) on the Pi (Nick, 2026-10-26).
#     cd ~/alvik-fleet-capstone && bash tools/pi/install_launcher.sh
# It: saves the camera names (which camera is the overhead, which the side), asks for a PIN for the page, sets up a
# systemd service (mirlt-launcher) that starts the page at every boot, and prints the page's addresses.
# Run it again any time: it keeps the PIN unless you ask to change it, and restarts the page with the newest code.
# To remove it:  sudo systemctl disable --now mirlt-launcher && sudo rm /etc/systemd/system/mirlt-launcher.service
set -u
cd "$(dirname "$0")/../.."
REPO="$(pwd)"
ME="$(id -un)"
PORT=8091
UNIT=/etc/systemd/system/mirlt-launcher.service

echo "=== Installing the start page for robot sessions ==="
need=""
command -v tmux >/dev/null || need="$need tmux"
command -v python3 >/dev/null || need="$need python3"
if [ -n "$need" ]; then
  echo "Installing:$need (the Pi may ask for its password)"
  sudo apt install -y $need || { echo "Could not install$need. Stopped."; exit 1; }
fi

echo
echo "--- 1 of 4: camera names ---"
python3 tools/pi/camnames.py --init
python3 tools/pi/camnames.py

echo
echo "--- 2 of 4: the PIN for the start page ---"
if python3 -c "import sys; sys.path.insert(0,'tools/pi'); import launcher; sys.exit(0 if launcher.Conf().has_pin() else 1)"; then
  read -r -p "A PIN is already set. Press Enter to keep it, or type n and press Enter to choose a new one: " ans
  if [ "$ans" = "n" ]; then python3 tools/pi/launcher.py --set-pin || exit 1; fi
else
  python3 tools/pi/launcher.py --set-pin || { echo "No PIN set. Stopped."; exit 1; }
fi

echo
echo "--- 3 of 4: the service that starts the page at every boot (the Pi may ask for its password) ---"
TMP="$(mktemp)"
cat > "$TMP" <<EOF
[Unit]
Description=Start page for robot sessions (tools/pi/launcher.py)
After=network-online.target
Wants=network-online.target

[Service]
User=$ME
WorkingDirectory=$REPO
Environment=PYTHONUNBUFFERED=1
ExecStart=/usr/bin/python3 $REPO/tools/pi/launcher.py --port $PORT
Restart=always
RestartSec=3
# a session started from the page lives in tmux: restarting the page must never stop it
KillMode=process

[Install]
WantedBy=multi-user.target
EOF
sudo cp "$TMP" "$UNIT" && rm -f "$TMP" || { echo "Could not write $UNIT. Stopped."; exit 1; }
sudo systemctl daemon-reload
sudo systemctl enable mirlt-launcher >/dev/null 2>&1
sudo systemctl restart mirlt-launcher || { echo "The service did not start: sudo journalctl -u mirlt-launcher -n 30"; exit 1; }

echo
echo "--- 4 of 4: checking it answers ---"
ok=0
for i in 1 2 3 4 5 6 7 8 9 10; do
  if python3 -c "import urllib.request,sys; urllib.request.urlopen('http://127.0.0.1:$PORT/', timeout=2)" 2>/dev/null; then ok=1; break; fi
  sleep 1
done
if [ "$ok" != 1 ]; then
  echo "The page does not answer yet. What the service says:"
  sudo journalctl -u mirlt-launcher -n 20 --no-pager
  exit 1
fi
echo
echo "=============================================================="
echo " DONE. The start page is running, and starts by itself at every boot."
echo " Open one of these on your phone or laptop and enter your PIN:"
python3 tools/pi/launcher.py --addresses --port $PORT
echo " While no session runs, the old live-page address (port 8090) shows the start page too."
echo "=============================================================="
