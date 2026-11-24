# Visit points with a list per robot (24 Nov 2026, build e24)

Each robot gets its own list of stickers and drives it in order, stopping at each. This is how Jay-Sun runs two robots
at the school table. It is all on Mission Control's page (the new page; `docs/how-to/mission-control-start.md` says how
to open it).

## Make the lists

1. Click **Visit points** in the bottom bar, then tick **A list per robot**.
2. A row per robot appears (Alpha, Bravo). The row marked **(adding)** is the one your clicks go to; click a robot's
   name to switch.
3. Add points any of three ways:
   - click stickers on the map (each robot's points and route are in its own colour, numbered `A1, A2 ... B1 ...`);
   - click stickers in the camera's picture (`Show camera: overhead | side (camera 2)` changes camera);
   - type both lists at once in the box above the map and press **Use it**:

     ```
     Alpha:P01,P02,P08;Bravo:P35,P36,P34
     ```

4. Click a point's chip in the bar to take it out. **Undo** removes a robot's last point, **Clear** empties its list.
5. **Save as a preset** keeps both lists under a name; **Presets...** brings them back. Your lists also stay through a
   reload of the page.

A sticker ringed **red** is one where the camera cannot read a roof tag: a robot sent there drives on its wheels alone.

## What happens in the run

- Names are given at the start by each robot's wiggle; a robot keeps its name at this site.
- **A robot named in a list that is not found** at the start: the others run their own lists first, then take its
  points. A red card says so.
- **A robot that is held during the run** (the camera lost it, its pose disagreed): its points **wait for it**. It
  takes them up when it rejoins. Another robot takes them only after finishing its own list, if the first is still
  held then.
- **A point that is missed** (the robot stopped too far from it, there was no room to turn, a new plan left it out) is
  said the moment it happens and is **tried once more at the end of the list**, by the other robot if its own is held.
- **A robot whose battery falls under the floor** (25 %) finishes the point it is on, drives to its home spot, and its
  remaining points go to the other robot.
- The run's **time limit** follows its plan: 4 times the planned time, and after that it ends only once no new point
  has been reached for a while. Nothing to set.

## How it ended

The card at the top right says `Done: all 40 points reached` or `Ended: 34 of 40 points reached`, then for each robot's
list which points were reached and, for every missed point, the reason and what to do next. The same lines are in the
terminal and in the run folder's `mission_summary.json` (`visit_result`).

**Run again** runs the same two lists again (the button shows them). **New run** goes back to the plan with the lists
still there.

## From the terminal instead (Pi window)

```
./run-fleet-pi.sh visit "Alpha:P01,P02,P08;Bravo:P35,P36,P34"
```

Add `dry` at the end to practise in the simulator.

## The old ways, if ever needed

Each of these is one switch in the Safety group (Settings > Safety, or `docs/how-to/safety-group.md`):
`MIRLT_HELD_POINTS=reassign` (a held robot's points go to the other at once), `MIRLT_POINT_RETRY=off` (a missed point
is not tried again), `MIRLT_BATTERY_LOW=finish`, `MIRLT_VISIT_TIME_LIMIT=fixed`.
