# Measuring the school Pi (the bench, e16)

What it does: about 20 minutes on the Pi, **no robot moves**. The robots are simulated (a practice run on the school's
real path, 3 robots, 40 points). The camera is real: the recorder runs, and a tag bridge reads its pictures and sends
nothing. It measures two setups one after the other:

- **old**: everything as before 11 Nov (video saved, preview always made, one picture encode per reader, motion
  detector on every picture);
- **trimmed**: the new defaults.

For each it logs every program's and every thread's CPU, the load, the clock, the temperature, the throttle flags,
how long tag detection takes per picture and how long re-plans take. It writes a short report and a side-by-side table.
If the second camera is plugged in (and its model is saved), it is measured too.

Before you start: the robots can stay off or asleep. The cover can stay as it is; write down whether it was on or off.

## 1. Log in to the Pi

On the laptop, open **Terminal** (Mac) or **PowerShell** (Windows) and type:

```
ssh nick@100.72.183.28
```

Press Enter. Type the Pi's password if it asks. You should see a line ending in `$`.

## 2. Go to the code

```
cd ~/alvik-fleet-capstone
```

## 3. Get the newest code

```
git pull
```

You should see `Fast-forward` and a list of files, or `Already up to date.`

## 4. Run the bench (both setups, 10 minutes each)

```
./run-fleet-pi.sh bench 10
```

It prints `Pi bench: both, 10 min each ... NO ROBOT MOVES` and then works for about 20-22 minutes. Leave the window
open. At the end it prints a table like this:

```
| What | old defaults | trimmed defaults |
| 1-min load, median | ... | ... |
...
```

If SSH drops, the bench stops. To run it so a closed window does not stop it, use step 4b instead.

## 4b. (Instead of 4) Run it in its own window that survives a closed SSH

```
tmux new-session -s bench "./run-fleet-pi.sh bench 10; read -p 'Bench done. Press Enter.'"
```

To leave it running: press **Ctrl+B**, then **D**. To look again later (after step 1 and 2):

```
tmux attach -t bench
```

## 5. Send the result

Copy the table it printed into the chat. The full numbers are in the folder it names at the end
(`logs/<date-time>-bench`); the chat can ask for it.

## Only one setup

The trimmed defaults alone, 10 minutes:

```
./run-fleet-pi.sh bench 10 trimmed
```

## After the bench

The recorder is left running with the new defaults (no video saved). Nothing else needs to be done.

## If something goes wrong

- `A run is open (tmux session ...)`: a run is still open. Close it first (`./run-fleet-pi.sh stopall`), then step 4.
- `no camera model at .../tagcal/overhead.json`: the real camera's tag bridge is left out; the rest is still measured.
  Run Mission Control's Setup, Measure once, then the bench again.
- `Stopped: no overhead camera found by its name`: the camera is not plugged in or not seen. Check the USB cable.

What it does not do: drive any robot, change any setting on a robot, or delete anything.

## (e17) Comparing the core layouts

Which of the Pi's 4 cores each program uses is now a setting, `MIRLT_CORE_LAYOUT`:

| Layout | Recorder | Planner (free roam, run_plan, mission_run) | Camera bridge |
|---|---|---|---|
| A (as up to 11 Nov) | cores 0-1 (one camera: 1) | 2-3 | 2-3 (shared with the planner) |
| **B (the default: Nick chose B on 12 Nov, pending the bench)** | 0 | 1 | 2-3 |
| C | any | any | any |

To measure all three one after the other (about 3 x the minutes you give; nothing moves):

```
./run-fleet-pi.sh bench 10 layouts
```

(e18) Nick chose B on 12 Nov as the default, pending the bench: it stays the default until the bench's numbers say
otherwise. A and C stay available behind `MIRLT_CORE_LAYOUT`.

The table at the end adds: pictures the real camera's bridge handled a second, the delay from picture to result at the
bridge, and the practice run's watchdog stops. Paste it into the chat. To use a layout for every run until the Pi is
restarted, type (for example) `export MIRLT_CORE_LAYOUT=A` before `./run-fleet-pi.sh ...`.

Note: the bench's real-camera bridge is the roof-tag bridge (school). At home (no roof tags) the bench measures the
practice run only; there the decisive numbers come from one short free roam per layout (`cpu.csv`, `camera.csv`'s
delay, the watchdog lines).
