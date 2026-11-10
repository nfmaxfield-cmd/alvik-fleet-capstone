# The USB stick (FLEET) at start-up

The runs save their frames and the recorder's video on the USB stick, not on the Pi's small card. The stick is
formatted ext4 and labelled **FLEET**; it is normally mounted at `/media/nick/FLEET`. Since build e12,
`./run-fleet-pi.sh mc` prints a warning starting with `!! WARNING: the USB stick FLEET is NOT mounted` when it is
missing.

## Mount it now (any time, safe to repeat)

In the **terminal window connected to the Pi**, type this and press Enter. If the Pi asks for its password, type it
(nothing shows while you type) and press Enter:

```
tools/pi/mount_fleet.sh
```

You should see `Done: ... free on /media/nick/FLEET`, or `nothing to do` if it was already mounted.

## Make it mount by itself at every start-up (once)

This adds one line to the Pi's `/etc/fstab`. Claude does not edit system files; you add it yourself. `nofail` means the
Pi still starts normally when the stick is not plugged in.

1. In the **terminal window connected to the Pi**, make the folder (press Enter after it):
   ```
   sudo mkdir -p /media/nick/FLEET
   ```
2. Add the line (press Enter after it; nothing is printed except the line itself):
   ```
   echo 'LABEL=FLEET /media/nick/FLEET ext4 defaults,nofail,x-systemd.device-timeout=10s 0 2' | sudo tee -a /etc/fstab
   ```
3. Check the line is right without restarting. You should see no error, and the next command shows the stick:
   ```
   sudo mount -a
   ```
   ```
   findmnt /media/nick/FLEET
   ```
4. Let your user write to it (once):
   ```
   sudo chown nick: /media/nick/FLEET
   ```

If step 3 prints an error, open the file and delete the last line: `sudo nano /etc/fstab`, move to the last line,
press Ctrl+K, then Ctrl+O, Enter, Ctrl+X.
