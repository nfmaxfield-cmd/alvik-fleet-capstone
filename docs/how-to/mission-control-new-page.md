# Mission Control's new page at home (12 Nov build): update the Pi, open it, run, and the SSH fallback

Mission Control now opens on **the table**: the live map is the page. Start, watch and STOP home runs from it; you no
longer need to start free roam over SSH. The SSH commands still work exactly as before (section 6) if the page is
ever not there.

Windows used below:
- **Laptop terminal**: PowerShell on Windows (Start menu, type `PowerShell`, press Enter) or Terminal on a Mac
  (Cmd+Space, type `Terminal`, press Enter).
- **Pi window**: the laptop terminal after you have logged in to the Pi (step 1). Its lines end in `$`.
- **Browser**: Chrome, Edge or Safari on the laptop.

Type one box at a time, then press **Enter**, and wait for the `$` before the next one.

## 1. Log in to the home Pi (laptop terminal)

Tailscale must be on (the icon in the corner of the screen shows `Connected`).

```
ssh st1maxfini@100.122.222.75
```

Type the Pi's password if it asks (nothing shows while you type), press **Enter**. You should see a line ending in
`$`. This window is now the **Pi window**.

## 2. Get the new version (Pi window)

```
cd ~/alvik-fleet-capstone
```

```
git pull
```

You should see `Fast-forward` and a list of files (or `Already up to date.`).

## 3. Restart Mission Control (Pi window)

```
./run-fleet-pi.sh mc-stop
```

You should see `Mission Control is closing` or `Mission Control is not running`.

```
./run-fleet-pi.sh mc
```

Within about 10 seconds you should see `Mission Control is open: http://100.122.222.75:8095 in the laptop's browser`.
You can close the Pi window now; Mission Control keeps running.

## 4. Open the page (browser)

Type this in the address bar and press **Enter**:

```
http://100.122.222.75:8095
```

You should see **Mission Control** at the top left, the board in the middle, a **Pi health** box on the right and a bar
along the bottom with **Calibrate / Free roam / Visit points / Experiment routes** and a blue **Start** button.

- **Sound**: click **Sound: off** at the top once. It turns green (**Sound: on**) and plays a short beep. From then on a
  robot that needs a hand plays three beeps, again every 20 s while it still needs one. Browsers only allow sound after
  a click, so after reloading the page it may say **Sound: click to enable**: click it once.
- **The old page** is still there: **Settings** (top right) > **Open the old page**, or `http://100.122.222.75:8095/classic`.

## 5. A home free roam (browser)

1. Put the robots on the board and switch them on.
2. In the bottom bar click **Free roam**, then **Start Free roam**.
3. Answer the two questions (OK / Cancel). You should see the robots appear on the map by their nicknames
   (**Left**, **Center**, **Right**), each in a ring, and their cards on the right.
4. To stop: the red **STOP** (top right, or **STOP every robot** in the bottom bar). The bar says **Stopping...** and
   lists each robot until every one has answered; then **Stopped: every robot confirmed ...**.
5. When it ends, the bar shows **Uploading...** then **Uploaded** with a link to the release (uploads are on by
   default; **Settings** > **This run** to turn them off).

## 6. The SSH fallback (Pi window)

If the page does not open, start free roam the old way. Log in first (step 1), then:

```
cd ~/alvik-fleet-capstone
```

```
./run-fleet-pi.sh freeroam
```

It runs in the tmux session `freeroam` and uploads at the end, as before. To stop it:

```
./run-fleet-pi.sh freeroam-stop
```

You should see `Sent Ctrl+C to the free roam: every robot stops ...`. Both ways now also write `pi_monitor.csv` (the
Pi's health every 2 s) into the run's folder.
