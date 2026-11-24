> **Archived 24 Nov 2026 (e24).** This guide describes the OLD page (now at `/classic`) from a Mac. The current guide is `docs/how-to/mission-control-start.md` (the new page, Windows PowerShell). Kept as it was below.

# Mission Control's Start screen (build e07, 6 Nov): Monday at the school table

Four big buttons and a red **STOP**. Nothing to type. Mission Control finds by itself whether it is at home, at
school or somewhere new, finds the robots by their wiggle, and measures the table with the camera.

Windows used below:
- **Mac Terminal**: the Terminal app on the Mac (Spotlight: press Cmd+Space, type `Terminal`, press Enter).
- **Pi window**: the Mac Terminal after you have logged in to the Pi (step 2). Its lines end in `nick@rfidthesis:~ $`.
- **Browser**: Chrome or Safari on the Mac.

Type one box at a time, then press **Enter**.

## 1. Tailscale on the joincapstonedb account (Mac)

1. Top right of the Mac's screen, click the **Tailscale** icon.
2. Click the account name at the top of its menu and choose **joincapstonedb**.
3. You should see a tick next to it and `Connected`.

## 2. Log in to the Pi (Mac Terminal)

```
ssh nick@100.72.183.28
```

Type nick's Pi password (nothing shows while you type), press **Enter**. You should see a line ending in
`nick@rfidthesis:~ $`. This window is now the **Pi window**.

## 3. Get Monday's version (Pi window)

```
cd ~/alvik-fleet-capstone
```

```
git pull
```

You should see `Fast-forward` (or `Already up to date.`).

## 4. Restart Mission Control (Pi window)

Only needed once after the `git pull` (an old Mission Control may still be running from last week):

```
./run-fleet-pi.sh mc-stop
```

You should see `Mission Control is closing` or `Mission Control is not running`. Then:

```
./run-fleet-pi.sh mc
```

You should see, within about 10 seconds:
- `This looks like the site school-conference ...` (only if it had to switch), and
- `Mission Control is open: http://100.72.183.28:8095 in the laptop's browser`.

## 5. Put the robots down (at the table)

1. Switch on every robot you want to use. Any number works (1, 2, 3 ...).
2. Put each one on the table at the **TV end** (where the camera sees it), at least a hand's width from the edges and
   from each other. Nose direction does not matter.
3. Jay-Sun must be in the room (the robots only drive at school while he is there).

## 6. Open the page (Browser)

1. Go to `http://100.72.183.28:8095`
2. You should see the **Start** screen:
   - a red bar at the top saying `ROBOTS: the real robots` (if it says `PRACTICE`, the Pi's camera is not answering:
     see the table at the end),
   - **Place**: `School: the conference table`,
   - **Robots**: `The camera sees 2 roof tags: 2 robots; each is named (Alpha, Bravo, ...) by its wiggle when a run
     starts` (the number is how many robots you put down),
   - **Area**: `Using the area the camera can see ...`,
   - **Table**: `Table measured by the cameras: 396 x 168 cm` (or `The cameras measure the table by themselves at
     the first run`),
   - four big buttons: **Calibrate**, **Free roam**, **Visit points**, **Experiment routes**.

If a big **yellow** card says `One thing to do first: click the table's corners`, press **Show me the picture to
click** and click the four corners in the order the page shows. This only appears when the camera cannot see the
corner tiles; at school it should not appear.

## 7. Run something (Browser)

1. Press one big button, for example **Visit points**.
2. A box asks `Start Visit points on every robot on the table?`: press **OK**.
3. A second box asks `Is he here?` (Jay-Sun): press **OK**.
4. You should see a status box above the buttons, for example `Visit points is running.` with the time, and lines
   such as `Found: Alpha at ..., Bravo at ...`, each robot wiggling, `TABLE SHARING: ...`, `SKIPPED (n points): ...
   the camera does not read tags there` (the far half of the table: expected with one camera).
5. The top of the page says `Visit points · 2:15` while it runs.

What each button does:

| Button | What you will see | How long |
|---|---|---|
| **Calibrate** | Each robot in turn: `Naming each robot by its wiggle`, `Measuring each robot's wheels and turns`, `Checking the fix on a tour`, `Parking`, `Done` | about 40 min a robot |
| **Free roam** | Every robot drives at once on smooth curves between points in its share of the table | about 10 min (Advanced changes it) |
| **Visit points** | Every robot drives to its points in straight lines, turning on the spot | 5-15 min |
| **Experiment routes** | Jay-Sun's next route (the button says which: `next is Experiment 1: in order`) | 5-10 min |

**Experiment routes with one camera:** the routes use the whole table, and the camera reads only the TV-end half. The
run then stops before anything moves and says which points the camera cannot read. That is Jay-Sun's agreement
(half the table needs Dr. Sodhi's OK). With his OK: open **Advanced** at the bottom and tick **Experiments may use
the half the camera cannot read (Dr. Sodhi agreed)**, then press the button again.

## 8. Stop (Browser)

Press the red **STOP** at the top right (it is on every screen), or **STOP every robot** in the status box. Every
robot stops where it is. You should see `STOP sent: ...` under the buttons.

When a run ends, the robots are **sent to sleep right away** (the status box says so). To wake them for the next run
you do nothing: the next run wakes them. To keep them awake instead: **Advanced**, untick **Robots sleep as soon as a
run ends**.

## 9. Practice without robots (any computer)

The same screen runs the simulator when no real camera answers (`PRACTICE` in the blue bar at the top). To practise
on purpose, **Mac Terminal** (not the Pi window), in the repo folder:

```
python3 tools/fleet/mission_control.py --sim-room school
```

then **Browser**: `http://127.0.0.1:8095`. `--sim-room home` and `--sim-room new` show the other places. Practice runs
break things on purpose (roof tags turned, a dark patch, the camera's clock off, a robot restarting) so you can see
the software cope.

## Advanced (only if needed)

Closed by default, at the bottom of the Start screen. Nothing in it is needed to run.

- **Typed values**: table length and width, roof tag height. These are first guesses only: the camera measures them,
  and when the two disagree the camera wins and the difference is logged (`tagcal/disagreements.jsonl` on the Pi).
  Saved on this Pi only.
- **Firmware check**: `No check (old firmware allowed)` is the old `--min-firmware ""` override.
- **Corner clicking**: automatic by default (only when no corner tile is seen).
- **Line sensor marks with no drop**: logged only (Nick, 6 Nov). The e06 way is kept as a choice.
- **Place**: `Always use school-conference` pins the site (the old `site` command); automatic otherwise.
- **Show all the older screens**: brings back Connect ... Production.

## If something looks wrong

| You see | Why | Do |
|---|---|---|
| Blue `PRACTICE` bar at the Pi's address | The Pi's camera recorder is not running | **Pi window:** `./run-fleet-pi.sh mc-stop`, then `./run-fleet-pi.sh mc` |
| `Place: home` at school | No camera was found when Mission Control started | Plug the C920e in, then **Pi window:** `./run-fleet-pi.sh mc-stop`, then `./run-fleet-pi.sh mc` |
| `No robot seen on the table yet` | The robots are off, or outside the camera's view | Switch them on; move them to the TV end |
| A button is grey with a reason | That run cannot work here (the reason says why) | Read the reason; at home, Visit points and Experiment routes are grey (no points on the board) |
| `A run is going: press Stop first` | Another run is still going | Press **STOP**, wait 10 s, press the button again |
