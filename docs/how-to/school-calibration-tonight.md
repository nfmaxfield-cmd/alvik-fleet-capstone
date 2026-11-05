# At school: the calibration run (zones 1-2)

Updated 5 Nov, 11:00 answers (build e04). At the school conference table. One robot, **Alvik 3** (WiFi name AL-1DC0),
stands on sticker **P11**, maybe **asleep** (it wakes for 20 s every 10 minutes). Its roof tag is id 2. Nobody needs to
type which tag is on which robot: the run finds out by itself (the robot wiggles; the tag that wiggles with it is its
tag), and from then on remembers it.

New at 11:00 on 5 Nov (build e04):

- **Spots: P11, then P01, then P09.** The second robot is calibrated and parked on **P01** (the corner sticker at the
  TV end, by the edge with tags 20 and 24) instead of P05; a third robot on **P09** (by the whiteboard edge, the TV
  end). A robot that stands somewhere else is driven to its spot first.
- **The first calibration of each robot starts from the defaults** (wheel_scale 1, gyro_scale 1, gyro_drift_dps 0),
  so phase A measures the robot uncalibrated. Its next calibration starts from the values saved on it. The card and
  the Pi window say which (`PHASE A STARTS FROM DEFAULTS` or `... FROM ITS SAVED VALUES`). In phase A an uncalibrated
  robot's turns can be several degrees off: the camera keeps it off the edges (it corrects the heading before any
  drive that would come too close).
- **Robots must be on firmware 2026-11-05a or newer** (the repo has 2026-11-05b since the home free-roam build; either
  is fine). A robot on older firmware is **refused** with a line saying so (it is
  not moved, and the others keep out of its way). Reflash it (section 11 or 12), then start again.
- **Robots are no longer renamed** at school: each keeps its own name (AL-xxxx); Alpha / Bravo exist only inside the
  run.
- **Each robot's roof tag is remembered** (on the Pi) once a wiggle has found it. The next runs only check it: a
  shorter wiggle (45 degrees and back). If a different tag turns, the run says `PIN CHECK FAILED` and uses the tag that
  did turn.
- **Flashing over WiFi from the Pi** (section 12), when your PC is not on the robots' WiFi.

Earlier on 5 Nov (build e03):

- **Every robot on the table is calibrated, one after another.** The first stands on **P11**, the others on their
  spots (P01, P09 since 11:00). While one drives, the others stand still at their spots and the driving robot keeps
  clear of them. Each one parks at its own spot and goes to sleep when it is done.
- **The new values are saved on the robot** with `save` (firmware 2026-11-05a), so they survive switching it off and
  its sleep. (The old `corr save` way for older firmware is still in the code, switched off: older firmware is
  refused now.)
- **Waking works from any address** (step 5): the robot is woken wherever it answers from.
- **The robot can turn after waking**: a robot woken from sleep has restarted and refused to turn until its compass
  check ran; the run now does that check first (a few degrees of wiggle).
- **The camera's name** is set for the school (Logitech C920e), and home's camera names and home's floor
  calibration are never used here.
- **Every run, not only calibration, starts with the wiggle** (Mission Control's runs and the experiments): each robot
  wiggles in turn, gets its tag, and is given its slot (Alpha / Bravo) by where it stands.

What the run does, about 40 minutes per robot, all at experiment speed (drive 10 cm/s, turn 60 deg/s, last 5 cm at 5 cm/s):

1. finds the robots on the WiFi and wakes them, checks each one's firmware (older than 2026-11-05a: refused), reads
   each one's current settings and saves them;
2. binds each roof tag (a quarter turn and back, one robot at a time; a robot whose tag is remembered: 45 degrees and
   back, as a check);
3. then for each robot in turn (the others standing at their spots):
   - **phase A**: from the defaults the first time (from its saved values after that): turns on the spot at its spot,
     straight rolls out and back, one minute standing still, then a tour of P01-P20 (skipping the other robots'
     spots), all on its own wheels and gyro; the camera only keeps it off the edges, the outlets, the other robots and
     the edge of its view;
   - **phase B**: works out its new wheel_scale, gyro_scale and gyro_drift_dps, sends them, reads them back and
     **saves them on the robot**;
   - **phase C**: three more tours of P01-P20 with the new settings (in order, reversed, shuffled);
   - parks at its spot, sends `sleep 10`;
4. writes the report (one section per robot).

Build reports: [e02-nov04-school-calibration-run](../build-reports/e02-nov04-school-calibration-run.md),
[e03-nov05-save-wiggle-multi](../build-reports/e03-nov05-save-wiggle-multi.md),
[e04-nov05-baseline-pins-ota](../build-reports/e04-nov05-baseline-pins-ota.md).

Three places you type or click, always named at the start of each step:

- **Mac Terminal**: the Terminal app on your Mac (Finder, Applications, Utilities, Terminal).
- **Pi window**: the same Terminal window after step 2, when it is logged in to the Pi (the line starts `nick@rfidthesis`).
- **Browser**: Chrome or Safari on your Mac.

Type each box as one line, then press **Enter**.

---

## 1. Tailscale on the joincapstonedb account (Mac)

1. **Mac**, top right of the screen: click the **Tailscale** icon in the menu bar.
2. Click the account name at the top of its menu. Choose **joincapstonedb** (the network of
   capstonedatabaseuri@gmail.com). If it is not listed: **Add account** and sign in with that Google account.
3. You should see a tick next to that account and `Connected`.

## 2. Log in to the Pi (Mac Terminal)

**Mac Terminal:**

```
ssh nick@100.72.183.28
```

The first time it asks `Are you sure you want to continue connecting (yes/no)?`: type `yes`, press **Enter**. Then it
asks for nick's password on the Pi: type it (nothing shows while you type) and press **Enter**. You should see a line
ending in `nick@rfidthesis:~ $`. From here on this window is the **Pi window**.

## 3. Get our repository onto the Pi (Pi window)

**Pi window:**

```
git clone https://github.com/nfmaxfield-cmd/alvik-fleet-capstone.git ~/alvik-fleet-capstone
```

You should see `Cloning into ...` and `done.`

**Pi window:**

```
cd ~/alvik-fleet-capstone
```

Only if the clone said `already exists` (it was cloned before), get tonight's version. **Pi window:**

```
git pull
```

You should see `Fast-forward` or `Already up to date.`

## 4. Set the Pi up (Pi window, about 2-10 minutes)

**Pi window:**

```
./run-fleet-pi.sh school-setup
```

It checks the programs and Python packages, installs what is missing (it may ask for nick's password once: type it,
press **Enter**), sets the camera's focus, picks the school site, and runs the self-check. You should see, near the end:

- `PASS  camera focus: /dev/video0 autofocus off, focus_absolute 0`
- a list of `PASS` / `WARN` lines, and
- `School setup DONE: no FAIL lines.`

Also, a few lines above the self-check, the camera names:

- `Camera names saved in .../school-conference/cameras.json from the site school-conference's camera_names
  (overhead: C920e; side: none, switched off).` (or `Camera names already set ... (kept)` if you set it by hand), then
- `OK: overhead camera: Logi Webcam C920e at /dev/video0` and `OFF: the side camera (camera 2) is switched off here`.

`WARN` lines for the GitHub token (uploads), `roof height` (6.19 cm is typed per robot; the run uses it) and `video
space` are fine. A `FAIL` line: see **If something is different** below.

## 5. Wake the robot (Pi window, up to 10 minutes)

**Pi window:**

```
python3 tools/pi/robot_sleep.py wake any
```

You should see `Waking any robot, from any address ...`, then within 10 minutes `heard AL-1DC0 at 192.168.1.xxx,
checking in: sending it wake` and `AL-1DC0: awake at 192.168.1.xxx, battery NN %`. Then the prompt comes back. Write
down the battery: under 25 % the run will not start; charge it first.

With two robots on the table, wake both (it waits for the second one's check-in too). **Pi window:**

```
python3 tools/pi/robot_sleep.py wake any --count 2
```

With three robots. **Pi window:**

```
python3 tools/pi/robot_sleep.py wake any --count 3
```

To wake one robot by its name, wherever it is on the WiFi. **Pi window:**

```
python3 tools/pi/robot_sleep.py wake --name AL-1DC0
```

(The old way, `wake 192.168.1.134`, still works but only listens to that one address; it now tells you if it hears
the robot somewhere else. If you skip this step, the run itself keeps waking the robots for up to 12 minutes.)

## 6. Start Mission Control (Pi window)

**Pi window:**

```
./run-fleet-pi.sh mc
```

You should see `Mission Control is open: http://100.72.183.28:8095 in the laptop's browser` and `It keeps running if
this window closes.` It also sets the camera's focus again (`PASS  camera focus ...`).

## 7. Open the page and Measure (Browser)

1. **Browser**: go to `http://100.72.183.28:8095`
2. Click **Connect**, then **Robots**.
3. Click **Cameras & table** in the top bar. Press **Check the cameras**: every line PASS (a WARN or
   FAIL about camera 2 / "side" is expected tonight: only one camera is plugged in).
4. Press **Take pictures**. You should see orange outlines on tags **20, 23, 24 and 25** (the two corners at the TV end
   and the two side tags). **21 and 22 are expected to be missing** (too small from the TV end). Do **not** click
   corners or stickers.
5. Press **Measure**. You should see the table near **396.2 x 167.6 cm** and a line like `overhead placed from 4
   reference tags (20, 23, 24, 25)`.

## 8. Run the calibration (Browser)

1. Click **Production** in the top bar.
2. In the card **Calibration (zones 1-2)**, press **Start calibration on the robot**, then **OK**.
3. You should see, within a minute: `Finding the robot`, maybe `Compass check on AL-1DC0` (the robot wiggles a few
   degrees: expected after it slept), then `Binding its roof tag`, then `Robot: AL-1DC0, roof tag 2`. The robot turns
   a quarter turn left and back: **that is the wiggle, expected** (45 degrees and back once its tag is remembered).
   With two or three robots, each does its wiggle in turn, and the card lists `Robots: AL-1DC0 at P11 (tag 2),
   AL-xxxx at P01 (tag n), AL-yyyy at P09 (tag m)`; the others first drive to their spots and wait there.
   If the card shows **Refused:** with a robot's name and its firmware: that robot is not moved. The others carry on.
   Reflash it later (section 11 or 12).
4. Then a line `Phase A from the defaults 1 / 1 / 0: the first calibration of AL-1DC0 on this site` (or `from its
   saved values` on a later calibration), then `Phase A ...`. The robot turns on the spot 12 times, rolls out and back,
   stands still for a minute, then visits P01-P20. Then `Phase B`, and a line `Saved on the robot: saved on the robot
   with save (firmware 2026-11-05a) and checked`. Then `Phase C` (three tours), then `Parking`, then the same for the
   next robot, then `Done`.
5. You may close the browser; the run goes on. Open the address again to see where it is.

**To stop the robot at any moment** (it heads for an edge; pillows are on the floor), any one of:

- **Browser**: press **Stop** in the Calibration card.
- **Pi window** (open a second Mac Terminal and do step 2 again if the first is busy), first:

```
cd ~/alvik-fleet-capstone
```

  then:

```
./run-fleet-pi.sh stopall
```

After a stop, the robot stays where it is (it is not driven back). Put it back on its spot by hand (P11; a second
robot on P01, a third on P09), nose towards the whiteboard, before starting again.

## 9. Read the result (Browser)

When the card says **Done**: read the plain-words line under it (the new settings, whether they read back, and
whether they were saved on the robot), then press **Show the report**. It has the before and after errors per leg, the fitted settings, Jay-Sun's four points
(P01, P05, P11, P15) with and without the height correction, the battery used, and the two closing lists.

## 10. Send the files (Pi window): start this straight away

Start this as soon as the run says **Done**, so it uploads while you read the report. **Pi window:**

```
./run-fleet-pi.sh calib-upload
```

You should see `Uploading .../calib/<date-time>` and, at the end, the release's address on GitHub.

If it says there is no GitHub token, copy the folder to your Mac instead. First, **Pi window**, to see the folder's
name:

```
ls -td ~/alvik-fleet-capstone/logs/mission-control-school-conference/calib/* | head -1
```

Then **Mac Terminal** (a new window, not the Pi window; put the folder it printed in place of FOLDER):

```
scp -r nick@100.72.183.28:FOLDER ~/Desktop/
```

You should see the files copied, then the folder on your Desktop.

The robot is asleep at the end (`sleep 10`). To wake it later from home: steps 1 and 2 again, then **Pi window:**

```
cd ~/alvik-fleet-capstone
```

and step 5 again.

---

## 11. Reflash a robot with the new firmware by USB cable (Jay-Sun, once per robot; laptop with the Arduino IDE)

This one cable flash per robot also switches on WiFi flashing for every later flash (section 12), so after it Nick
can update the robots from home through the Pi.

Every robot must be on 2026-11-05a or newer before a run (older firmware is refused; the repo's firmware is
2026-11-05b today). The new firmware adds `save`, `unsave`,
`saved` and `id`, and its start-up line says which values came from saved storage. Whoever has the robots and the
laptop with the Arduino IDE and the robots' `secrets.h` does this (Nick, or Jay-Sun for the borrowed robots). One
robot at a time, about 5 minutes each. Nothing is lost: the saved values stay on the robot through the reflash.

**Before you build, add the WiFi-update password to your `secrets.h`** (Jay-Sun):

1. Open `firmware/home_lab/secrets.h` (your own copy, never in the repo) in Notepad.
2. Check the school WiFi lines (`WIFI_SSID2`, `WIFI_PASSWORD2`) are there and right, or the robot will not find the
   school WiFi after the flash.
3. At the end of the file, add one line, with a password **you choose** between the quotes:
   `#define OTA_PASSWORD "your-chosen-password"`
   (No line starting with `//` in front of it: `//` switches a line off.) Save.
4. Give that password to Nick yourself, privately (in person or a direct message). It is never written in the repo,
   in a report, or in any file of ours.

The start-up line in the Serial Monitor (step 6 below) then also says `AL-xxxx: WiFi firmware updates on`. If it says
`off`, the OTA_PASSWORD line was not read: check step 3 and upload again.

1. **Browser** on the laptop: go to `https://github.com/nfmaxfield-cmd/alvik-fleet-capstone`. Press the green
   **Code** button, then **Download ZIP**.
2. Open the downloaded `alvik-fleet-capstone-main.zip` (double-click) and drag the folder out to the Desktop.
3. Copy **your** `secrets.h` (the one you flashed with before; it is never in the repo or the ZIP) into the folder
   `alvik-fleet-capstone-main/firmware/home_lab/` on the Desktop. Never type the WiFi password anywhere else.
4. Plug the robot's board into the laptop with the USB-C **data** cable. Switch the robot **on**.
5. Arduino IDE: **File, Open**, `alvik-fleet-capstone-main/firmware/home_lab/home_lab.ino`. **Tools, Board**:
   `Arduino Nano ESP32`. **Tools, Port**: the one that appeared. Press **Upload** (the right arrow). Wait for
   `Done uploading`.
6. **Tools, Serial Monitor**, set `115200`. You should see `home_lab 2026-11-05b ready` (the repo's version today) and a line
   `AL-xxxx calibration at power-on: wheel_scale 0.9xxx (saved), gyro_scale 1.0xxx (saved), gyro_drift_dps ...
   (saved)` (or `(default)` for a robot never calibrated).
7. The next robot: steps 4-6.

To put a robot back to 1 / 1 / 0 (forget its saved values), **Pi window:**

```
python3 tools/fleet/robot_cmd.py --to AL-1DC0 --wait-s 3 "unsave"
```

(On the old firmware the same is `"corr reset"`.)

## 12. Reflash a robot over WiFi through the Pi (your PC is not on the robots' WiFi)

The Arduino IDE can only send firmware over WiFi to a robot on the same WiFi as the PC. At home your PC is not on the
school WiFi, but the Pi is. So: the PC builds the file, the file goes to the Pi through Tailscale, and the Pi sends it
to the robot. About 10 minutes the first time.

**It only works after Jay-Sun's cable flash with the OTA_PASSWORD line (section 11).** WiFi updates are ON only when
the `secrets.h` a robot was LAST built with had a line `#define OTA_PASSWORD "..."`. The earlier instructions for the
borrowed robots told Jay-Sun to change only the WiFi lines, so before his new cable flash **WiFi updates are most likely
OFF** on them. Step A tells you for sure; OFF means that robot needs section 11 first.

You need: the PC with the Arduino IDE; the repo folder on the PC with **your** `secrets.h` in
`firmware/home_lab/` with the school WiFi lines and the **same OTA_PASSWORD Jay-Sun chose** (he gives it to you
privately; type it into your own `secrets.h` only), so the next flash over WiFi works too; Tailscale on the PC switched
to the **joincapstonedb** account (as in step 1); a Pi window (steps 1-2).

**A. Are WiFi updates on? (Pi window, about 2 minutes; the robot restarts once, it does not move)**

```
cd ~/alvik-fleet-capstone
```

```
git pull
```

```
python3 tools/pi/ota_push.py check --name AL-1DC0
```

You should see `AL-1DC0: awake at 192.168.1.xxx`, then `Sending AL-1DC0 sleep 1 ...`, and about a minute later
either `WiFi firmware updates are ON on AL-1DC0`: go on with B, or `WiFi firmware updates are OFF`: stop here and use
the cable (section 11). `NOT SURE`: type the same command again.

**B. Build the file (PC, Arduino IDE)**

1. **Arduino IDE**: **File, Open**, then `firmware/home_lab/home_lab.ino` in the repo folder that has your
   `secrets.h`.
2. **Tools, Board**: `Arduino Nano ESP32`.
3. **Sketch** menu, **Export Compiled Binary**. Wait until the Output panel at the bottom says `Done compiling` (a
   red error instead: stop and send me the red lines).
4. **Sketch** menu, **Show Sketch Folder**. A File Explorer window opens. Double-click the folder `build`, then the
   one folder inside it (its name starts with `arduino.esp32`). You should see `home_lab.ino.bin` and some other
   `.bin` files. The one to send is **`home_lab.ino.bin`** (not `.merged.bin`, `.bootloader.bin` or
   `.partitions.bin`).
5. Hold **Shift**, right-click `home_lab.ino.bin`, choose **Copy as path**.

**C. Copy it to the Pi (PC, PowerShell)**

Start menu, type `PowerShell`, open **Windows PowerShell**. Type `scp ` (with a space), press **Ctrl+V** to paste
the path, then type ` nick@100.72.183.28:home_lab.ino.bin` (with a space before it). The line looks like this, with
your path:

```
scp "C:\Users\you\Desktop\alvik-fleet-capstone-main\firmware\home_lab\build\arduino.esp32.nano_nora\home_lab.ino.bin" nick@100.72.183.28:home_lab.ino.bin
```

Press **Enter**. The first time it asks `Are you sure you want to continue connecting`: type `yes`, **Enter**. Then
nick's password on the Pi. You should see `home_lab.ino.bin  100%` and a size of about 1000-1500 KB. This file holds
the WiFi passwords from your `secrets.h`: it goes only to the Pi and is deleted there in step F; never put it on
GitHub.

**D. Get espota.py onto the Pi (Pi window, once)**

espota.py is the small program from the ESP32 Arduino core that sends firmware over WiFi (about 15 KB; it is not in
our repo).

```
curl -fsSL -o ~/espota.py https://raw.githubusercontent.com/espressif/arduino-esp32/master/tools/espota.py
```

The prompt comes back with no message. Check it:

```
ls -l ~/espota.py
```

You should see a line ending in `espota.py` with a size of about 10000-20000. (The Pi's card has about 1 GB free; the
two files together are under 2 MB.)

**E. Send it to the robot (Pi window, about 1 minute)**

```
python3 tools/pi/ota_push.py push --name AL-1DC0 --bin ~/home_lab.ino.bin
```

You should see `AL-1DC0 at 192.168.1.xxx, firmware 2026-..., idle. Sending home_lab.ino.bin ...`, then it asks for
the robot's WiFi update password: type the OTA_PASSWORD that was in `secrets.h` when this robot was **last** flashed
(Jay-Sun's, the first time; nothing shows while you type), press **Enter**. A progress bar runs; the robot's lights turn blue, then green, and it
restarts. At the end: `DONE: AL-1DC0 now runs firmware 2026-11-05b (was ...)` (the version you built).
`NOT FLASHED`: the line says the likely reason (wrong password, updates off, robot asleep); the robot still runs its
old firmware, nothing is broken.

**F. Delete the file from the Pi (Pi window)**

```
rm ~/home_lab.ino.bin
```

## If something is different

| You see | What it means | Do |
|---|---|---|
| Step 5 never says `awake` (10 minutes) | The robot is off, flat, or not on the WiFi | Ask Jay-Sun to look at the robot: lights on? If it was switched off and on, it is awake now: go on with step 6 |
| `rec` or step 4 says `the overhead camera (USB 2.0 Camera) is not plugged in` | An old copy of the code (home's camera names) | **Pi window:** `git pull`, then `python3 tools/pi/camnames.py --set overhead "C920e"` You should see `Saved: the overhead camera is the one named 'C920e'` |
| `did not do its wiggle (refused ... no compass check)` | An old copy of the code: the robot restarted when it woke | **Pi window:** `git pull`, then start the run again |
| `REFUSED: AL-xxxx is on firmware 2026-10-29b; this run needs 2026-11-05a or newer` | That robot has old firmware | It was not moved; the others carry on. Reflash it (section 11 by cable, or 12 over WiFi), then start again |
| `this robot's firmware has no save` on the card | Only if the firmware check was switched off (calibration.min_firmware "") | The values were not saved (calibration.old_firmware_save is off). Reflash it |
| `PIN CHECK FAILED: pinned to tag 2, but tag 3 turned with it` | Someone moved a roof tag to another robot (or swapped the robots' tags) | Nothing: the run uses the tag that turned and remembers it. If no tag was moved, tell me |
| `PHASE A STARTS FROM DEFAULTS` for a robot you calibrated before | The Pi has no record of it (a calibration before 5 Nov 11:00, or on another Pi) | Expected once. To start from its saved values instead: tell me (the run's `--from-saved`) |
| A mission run says `roof tag N is on the table but no robot of this run turned it` | A robot is on the table that is not in this run (a third robot, or one asleep) | Take that robot off the table, or run with all robots on the table |
| `STOPPED: no robot answered on the WiFi` on the card | Same as above, during the run | As above, then press **Start calibration** again |
| `no roof tag is seen by the camera` | The camera does not see a tag where the robot stands | Check the robot is on P11 with its tag on top, face up. Look at the camera picture (Cameras & table, Take pictures): is the robot in it? |
| `no roof tag moved when AL-1DC0 did its wiggle` | The tag on the robot was not read while it turned (blur, light), or the robot did not turn | Watch the robot during step 8.3: did it turn? If yes, check focus (row below) and start again |
| `2 tags moved` | Someone moved a second tag, or a second robot moved | Keep other tags and robots still; start again |
| Step 7: corner tags 20 or 23 not outlined, or Measure says a corner is missing | The camera does not see that end of the table | Ask Jay-Sun to check the tripod has not moved; tags 20 and 23 must be in the picture. Measure again |
| The picture looks blurry, or `WARN camera focus` | The camera's focus reset (unplugged / Pi restarted) | **Pi window:** `python3 tools/pi/camera_focus.py` You should see `PASS camera focus`. Then Take pictures again |
| The robot heads for an edge | Its own odometry is off (that is what phase A measures), and the camera should step in before the edge | Press **Stop** (or `stopall`). Note the time and the point it was going to. Put it back on P11 |
| `HOLDING: the camera lost the roof tag` | The tag was not read for over 1 s while driving: the robot stopped (cautious) | Nothing: it carries on when the tag is seen again. Over 30 s: the run ends where it is. Check nothing covers the robot |
| `LOW BATTERY` | Under 25 % | It parks at P11 and stops the series. Charge it; the report shows what was done |
| A line with `NOT OK` after the new values | A setting did not read back after two tries: the old value was put back | Fine: the report says which one |
| `port 5005 is in use` | Another program talks to the robots (robot_sleep.py still running, or a Mission Control run) | Wait for robot_sleep.py to finish, or stop the other run, then start again |
