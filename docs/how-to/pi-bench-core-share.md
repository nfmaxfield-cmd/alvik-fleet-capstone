# Setting the detector's core share from data (home Pi bench, about 25 minutes, robots parked)

**Premise.** The trained detector runs on demand on the planner's core 1, capped at 25 % of that core
(`tools/fleet/tagfree.py`, DUTY). 25 % was chosen, not measured. Too small and a lost or standing robot is found
slowly; too large and the planner and the cameras slow down. This bench measures both on the home Pi and picks the
share.

**What it does** (`tools/pi/detector_share_bench.py`), for each share in turn, 5 minutes each:
1. **0 %** (the detector off): the baseline. Its 95th percentiles are the run's normal spread.
2. **10 %, 25 %, 50 %**: the detector on core 1, capped at that share, asked for one parked robot at a time as if it
   were lost.

Under each share it runs a normal run's load: the recorder (the real cameras) and a practice free roam (simulated
robots) pinned to the planner's core 1. The Pi monitor records picture lag per camera, the planner's loop time, every
core, temperature and throttling. The detector's benefit is the **time to find** a lost standing robot.

**The rule.** A share passes when its median picture lag and its median planner loop stay at or under the baseline's
95th percentiles, with no throttle flag the baseline did not have. The **largest share that passes** is chosen.

**Robots:** none move. Leave Left, Center and Right **parked on the board**, anywhere the overhead camera sees them,
at least 10 cm apart. They may be switched off.

Windows used below:
- **Laptop terminal**: PowerShell on Windows (Start menu, type `PowerShell`, press Enter) or Terminal on a Mac.
- **Pi window**: the laptop terminal after step 1. Its lines end in `$`.

Type one box at a time, press **Enter**, and wait for the `$` before the next.

## 1. Log in to the home Pi (laptop terminal)

```
ssh st1maxfini@100.122.222.75
```

Type the Pi's password if it asks, press **Enter**. You should see a line ending in `$`.

## 2. Get the bench (Pi window)

```
cd ~/alvik-fleet-capstone
```

```
git pull
```

You should see `Fast-forward` (or `Already up to date.`).

## 3. Check that the detector model is on the Pi (Pi window)

```
ls ~/.config/mirlt/detector/
```

You should see `detector-v1.onnx`. If you see `No such file or directory`, put the model on first (the steps are in
`docs/build-reports/2026-11-12-tag-free.md`, "Putting the model on the home Pi"), then come back.

## 4. Nothing else may be running (Pi window)

```
./run-fleet-pi.sh stopall
```

You should see lines ending with what it stopped (or that nothing was running). If Mission Control has a run going,
press its STOP first.

## 5. Start the recorder (Pi window)

```
./run-fleet-pi.sh rec
```

You should see that the recorder is running (its picture: `http://100.122.222.75:8000` in the browser shows the board
and the parked robots).

## 6. Run the bench (Pi window)

```
python3 tools/pi/detector_share_bench.py run --minutes 5 --shares 10,25,50
```

You should see `Detector share bench: shares 0 %, 10 %, 25 %, 50 %, 5 min each (about 20 min). Robots stay parked.`,
then one line per share with its start time. Leave the laptop open (or keep the Pi window open) for about 20 minutes.
At the end a table appears:

| share of core 1 | picture lag median / 95 % (s) | planner loop median / 95 % (ms) | ... | time to find median / 95 % (s) | ... | verdict |
|---|---|---|---|---|---|---|

and a line **Chosen: NN %.** with why. If the window closed, the results are in the newest `logs/*-share-bench` folder:

```
python3 tools/pi/detector_share_bench.py report $(ls -td logs/*-share-bench | head -1)
```

## 7. Use the chosen share for every run (Pi window)

Replace `25` with the chosen number:

```
python3 tools/pi/detector_share_bench.py apply 25
```

You should see `Saved: every run now caps the detector at 25 % of one core (...share.json)`. Every run after this reads
it; the run's `state.json` shows it under `tagfree` > `load` > `duty_from: share.json (the bench's choice)`. To go back
to the built-in 25 %, delete the file:

```
rm ~/.config/mirlt/detector/share.json
```

## 8. Send the results

Copy the table from step 6 into the chat. The full numbers are in the bench folder (one `share-NN` folder per share,
each with `pi_monitor.csv` and `finds.json`).

## Notes and limits

- The practice free roam brings its own simulator, which a real run does not have: the bench's load is a little higher
  than a real run's. The baseline has the same simulator, so the comparison is fair; the absolute numbers are not a
  real run's.
- The parked robots' spots are learnt from the detector's own detections in the first minute of the first share above
  0 (that minute counts no finds).
- [chosen] numbers: 5 minutes a share, an ask every 2 s, "found" within 5 cm, "not found" after 20 s. None needs a
  designed experiment; the shares themselves (10 / 25 / 50) are the experiment. A finer second pass (for example 20 /
  30 / 40) is worth it only if the choice lands at a boundary.
