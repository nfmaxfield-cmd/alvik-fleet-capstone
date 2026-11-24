# Mission Control at the school table: start it, run Visit points, stop (updated 24 Nov 2026, build e24)

One page runs everything: the table seen from above, the robots on it, a bar at the bottom to choose and start a run,
and a red **STOP** that is always there. Nothing has to be typed to run. This guide is for the **new page** (the one
that opens by default). The page as it was before 12 Nov is still at `/classic` (Settings > Open the old page); its old
guide is `docs/archive/mission-control-start-e07-old-page.md`.

Windows used below:
- **PowerShell**: on the Windows laptop, press the Windows key, type `PowerShell`, press Enter. A blue or black window
  opens with a line ending in `>`.
- **Pi window**: the same PowerShell window after you have logged in to the Pi (step 2). Its lines end in `$`.
- **Browser**: Chrome or Edge on the laptop.

Type one box at a time, press **Enter**, and wait for the `$` (or `>`) to come back before the next box.

## 1. Tailscale on the school's account (laptop)

1. Click the Tailscale icon near the clock (bottom right; it may be behind the small `^` arrow).
2. If the account shown is not **joincapstonedb**, click the account name, then **joincapstonedb** (or "Log in to a
   different account" and sign in as joincapstonedb).
3. You should see `Connected`. If it says `Disconnected`, click **Connect**.

## 2. Log in to the school Pi (PowerShell)

```
ssh nick@100.72.183.28
```

Type the Pi's password if it asks (nothing shows while you type), press Enter. You should see a line ending in `$`.
This window is now the **Pi window**. If it says `Connection timed out`, Tailscale is on the wrong account (step 1).

## 3. Go to the code (Pi window)

```
cd ~/alvik-fleet-capstone
```

You should see the line change to end in `~/alvik-fleet-capstone $`.

## 4. Get the newest version (Pi window), only when there is one

```
./run-fleet-pi.sh update
```

One command does all of it: it closes Mission Control, backs up anything that was changed on this Pi, gets GitHub's
newest code, checks it, and starts Mission Control again. Wait for the last lines. You should see a line starting with
`PASS:`. If you see `FAIL:`, the line under it (`What to do:`) says what to type; the Pi is left exactly as it was.

## 5. Open Mission Control (Pi window)

```
./run-fleet-pi.sh mc
```

You should see `Mission Control is open: http://100.72.183.28:8095 in the laptop's browser`. It keeps running if you
close this window. (If it says `already open`, that is fine.)

## 6. Put the robots down (at the table)

1. Switch on each robot you want to use.
2. Put each on the table at least a hand's width from the edges, the outlet plates and each other. Which way it faces
   does not matter.
3. Jay-Sun must be in the room: at school the robots drive only while he is there.

## 7. Open the page (Browser)

Go to `http://100.72.183.28:8095`

You should see:
- at the top: **Mission Control**, the place (`School conference table ...`), a red pill `Robots (real)` and the red
  **STOP** button;
- under it one plain line, for example `Nothing is running. Robots: Alpha found; Bravo found.`;
- the table from above with its stickers (P01 ... P40) and the grey outlet plates;
- at the bottom: **Calibrate**, **Free roam**, **Visit points**, **Experiment routes** and a blue **Start** button.

If a red band across the top says `Mission Control is not answering`, the page has lost the Pi: do step 5 again.

## 8. Run Visit points

1. Click **Visit points** in the bottom bar.
2. Choose the points. Three ways, use any:
   - click stickers on the map, in the order to visit;
   - click stickers in the camera's picture beside the map (`Show camera: overhead | side (camera 2)` switches
     camera);
   - type the order in the box above the map and press **Use it**: `P01,P05,P11` for one list, or
     `Alpha:P01,P02,P08;Bravo:P35,P36,P34` for a list per robot.
   Leave everything empty and Start shares all the points out between the robots. A list per robot is explained in
   `docs/how-to/visit-points-lists.md`.
3. Click **Start Visit points**.
4. A panel asks once: tick **Jay-Sun is in the room** (needed) and the clear-space line, then click **Start**. It
   remembers your ticks until Mission Control is restarted, so **Run again** does not ask a second time.
5. While it runs the bar shows the time, **`3 of 6 points`** and a plain line (`Finding the robots`, `Naming each robot
   by its wiggle`, `Driving`). Stickers turn green with a tick as they are reached. **Log** opens the whole run's log.
6. If something needs you, a **red card** appears at the top right (for example `No pictures from the overhead camera`
   or `Every robot is held`) and the line under the header turns red. Each robot's card says why it is held.

## 9. When it ends

A card at the top right says how it went:
- `Done: all 6 points reached`, or
- `Ended: 5 of 6 points reached`, then for each missed point **why** and **what to do next**.

**Run again** (its name shows the lists it will run) starts the same lists again. **New run** goes back to the plan
with your lists still there, to change them.

## 10. Stop

Click the red **STOP** (top right, always there) or **STOP every robot** in the bar. Each robot's pill turns green as
it confirms; you should see `Stopped: every robot confirmed`. If a robot does not answer, the bar says so and why
(asleep, off, or out of WiFi) and **STOP again** sends it again.

If the page itself does not answer, stop the robots from the Pi window:

```
./run-fleet-pi.sh stopall
```

## 11. Practice without robots (any computer with the code)

```
python3 tools/fleet/mission_control.py --dry --sim-room school
```

Then open `http://127.0.0.1:8095`. The top says `Practice (simulator)`; nothing real moves and nothing is asked.

## If something looks wrong

| What you see | What to do |
|---|---|
| A red band `Mission Control is not answering` | Pi window: `./run-fleet-pi.sh mc` |
| `Not started, nothing moved: no such sticker on this site: P99` | Fix the typed order (stickers are P01 to P40) |
| A red card about the Pi's card being nearly full | Pi window: `./run-fleet-pi.sh space --move` (finished runs go to the USB drive; nothing is deleted). From e24 this happens by itself at each start when the USB drive is plugged in |
| A robot named in the order `was not found` | Switch that robot off and on, then Run again; meanwhile the other robot takes its points after its own |
| `Unknown command 'mcstop'. Did you mean mc-stop?` | Type the suggested command |
| The commands, in one screen | Pi window: `./run-fleet-pi.sh help` |
