# School runs copied to the home Pi, then to GitHub

Build e13 (10 Nov 2026). Jay-Sun allowed sharing the school Pi (**RFIDTHESIS**, on the school Tailscale account,
address `100.72.183.28`, user `nick`) with Nick's home Tailscale account. Once this is set up, the **home Pi** copies
new school run folders every hour (logs always; pictures when the school Pi's USB stick is mounted) and sends each
finished run to GitHub from home. The school Pi never gets a GitHub key.

What the copy never does: it never deletes anything at either end; a home file replaced by a newer school version
(a run still being written) is kept under `~/school-runs/.replaced/`. Files with "rfid" in their name are never copied
(raw RFID data stays private). The script: `tools/pi/school_pull.sh`; its log: `~/.local/state/mirlt/school_pull.log`.

The setup is done once, in four parts. Each box is one command: type it (or paste it), then press **Enter**.

---

## Part A. Share RFIDTHESIS with the home Tailscale account (laptop browser)

1. On the laptop, open a web browser and go to `https://login.tailscale.com/admin/machines`. Sign in with the
   **school** Tailscale account (the one RFIDTHESIS is on). You should see a list of machines with **rfidthesis** in it.
2. On the **rfidthesis** row, click the **...** button at the right end of the row, then click **Share...**.
3. In the box that opens, choose **Invite via email** and type the email address of Nick's **home** Tailscale
   account, then click **Share**. (If there is only **Copy share link**, click it and keep the link for step 5.)
   You should see the share listed as "pending".
4. Sign out of the school account (top right: your name, then **Log out**).
5. Open the invite email (or paste the share link into the browser's address bar). Sign in with the **home**
   Tailscale account and click **Accept**. You should see "rfidthesis" added to the home account's machine list,
   with the address `100.72.183.28`.

---

## Part B. A key on the home Pi, copied to the school Pi (laptop terminal, logged in to the home Pi)

1. On the laptop, open **PowerShell**. Log in to the **home Pi**. You should see the home Pi's prompt
   (`st1maxfini@st1maxfini:~ $`):
   ```
   ssh st1maxfini@100.122.222.75
   ```
2. Check the home Pi can now see the school Pi through Tailscale. You should see a line with `rfidthesis` and
   `100.72.183.28` in it:
   ```
   tailscale status
   ```
   If it is missing, Part A step 5 was not accepted on the account this Pi uses; do it again.
3. Check rsync is there. You should see `rsync  version 3...`:
   ```
   rsync --version
   ```
   Only if it says "command not found", type this (it may ask for the home Pi's password), then repeat step 3:
   ```
   sudo apt install -y rsync
   ```
4. Make the key the copy will use. Press **Enter** at both questions it asks (no passphrase). You should see
   "Your public key has been saved in /home/st1maxfini/.ssh/school_pi.pub":
   ```
   ssh-keygen -t ed25519 -f ~/.ssh/school_pi -C home-pi-to-school-pi
   ```
5. Copy the key to the school Pi. If it asks "Are you sure you want to continue connecting", type `yes` and press
   Enter; then type the **school Pi's** password for `nick` (nothing shows while you type). You should see
   "Number of key(s) added: 1":
   ```
   ssh-copy-id -i ~/.ssh/school_pi.pub nick@100.72.183.28
   ```
6. Check the key works with no password. You should see the word `RFIDTHESIS` (or `rfidthesis`) and no password
   question:
   ```
   ssh -i ~/.ssh/school_pi nick@100.72.183.28 hostname
   ```
7. Check rsync is on the school Pi too. You should see `rsync  version 3...`:
   ```
   ssh -i ~/.ssh/school_pi nick@100.72.183.28 rsync --version
   ```
   If it says "command not found", ask Jay-Sun before installing anything on his Pi.

---

## Part C. A test copy (laptop terminal, logged in to the home Pi)

1. If you are not logged in to the home Pi already, open **PowerShell** on the laptop and log in. You should see
   `st1maxfini@st1maxfini:~ $`:
   ```
   ssh st1maxfini@100.122.222.75
   ```
2. Go to the project folder:
   ```
   cd ~/alvik-fleet-capstone
   ```
3. Get the new code. You should see a list of changed files, `tools/pi/school_pull.sh` among them:
   ```
   git pull
   ```
4. A dry test: it logs in and lists what it would copy, copying nothing. You should see "logged in to
   nick@100.72.183.28", a list of folder names starting `2026...`, and a line about the USB stick:
   ```
   tools/pi/school_pull.sh --test
   ```
5. The first real copy, without uploading. It may take several minutes the first time. You should end with
   "the school Pi's USB stick is not mounted" or "pictures copied", and no line starting `!!`:
   ```
   tools/pi/school_pull.sh --no-upload
   ```
6. See what arrived. You should see the school run folders (names starting `2026...`, and `mission-control-...`):
   ```
   ls ~/school-runs/logs
   ```
7. The first full run (copy, then upload each finished run). You should see lines "uploaded: ..." and at the end
   "done: N run folder(s) uploaded this time":
   ```
   tools/pi/school_pull.sh
   ```
   The releases appear on GitHub as `school-<run folder name>`.

---

## Part D. Every hour by itself (laptop terminal, logged in to the home Pi)

1. If you are not logged in to the home Pi already, open **PowerShell** on the laptop and log in. You should see
   `st1maxfini@st1maxfini:~ $`:
   ```
   ssh st1maxfini@100.122.222.75
   ```
2. Open the home Pi's list of timed jobs. If it asks which editor, type `1` (nano) and press Enter. You should see
   a file that starts with comment lines (`#`):
   ```
   crontab -e
   ```
3. Use the arrow keys to go to the very end of the file, and paste this one line (it runs the copy at 7 minutes past
   every hour):
   ```
   7 * * * * cd $HOME/alvik-fleet-capstone && tools/pi/school_pull.sh >/dev/null 2>&1
   ```
4. Save and close: press **Ctrl+O**, then **Enter**, then **Ctrl+X**. You should see
   `crontab: installing new crontab`.
5. Check it is there. You should see the line from step 3:
   ```
   crontab -l
   ```
6. After the next hour has passed, read what it did. You should see a line "school_pull ... logged in" with that
   hour's time:
   ```
   tail -n 20 ~/.local/state/mirlt/school_pull.log
   ```

To stop it later: `crontab -e`, delete that one line, save as in step 4.

---

## Notes

- Runs are uploaded once they have been quiet for 15 minutes (nothing written), so a run still going is copied but
  not uploaded until it has finished.
- Pictures are copied to `~/school-runs/stick/` but only the logs go to GitHub. To send the pictures too, put
  `SCHOOL_UPLOAD_PICTURES=1 ` in front of `tools/pi/school_pull.sh` in the crontab line.
- If the school Pi is switched off or Tailscale is off, the hourly copy says "cannot log in to the school Pi" in the
  log and tries again the next hour.
