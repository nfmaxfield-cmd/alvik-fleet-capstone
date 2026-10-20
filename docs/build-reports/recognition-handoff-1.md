# Build report: recognition handoff version (2026-10-20)

**Premise.** Make the recognition studio a finished product another university team can install and
use, in the first of two build stages (plan: `docs/build-plans/recognition-handoff-1.md`). Nick asked
for every check to run and for the build to finish even past 90% of the weekly usage meter.

**What was built** (commits cb95c2c..a7fda48, 36 files, about 10,000 lines added):

| Part | What a person gets |
|---|---|
| Polish | The name **Maxfield IRL Tracking** ("Identify. Recognize. Label."); a setup screen that asks what you track (the ready-made Alvik set is one choice); a short tour (a full one on request); a progress bar on every wait; keys for unsure (?), magnifier (M), brightness (B), full screen (V), work area (W); same-picture frames accepted together; "height varies"; a work area per camera; the second camera prepared in the background |
| Use the model | `track-with-model.bat` / `use_model.py`: runs a trained model on a recording, a video file, a camera or the Pi's stream, offline; writes positions to a CSV (floor cm when calibrated); a live page with a green/amber/red confidence per object |
| Share a project | One `.mxirl` file (labels, settings, models; videos optional); open it elsewhere and point it at the recordings |
| Investigate | Q on any spot: pictures of it over time, how much it changed, the biggest changes, the usual picture, a plain verdict, every label made there with change-all / reject-all / unsure-all, ignore this spot, the second camera as referee |
| Time saved | A record of every finished frame; seconds per frame and share accepted unchanged, by model and guess source, measured and estimated kept apart; CSV downloads for the final report |
| Also | Track timeline; add an ordinary video file (phone or webcam) as a recording; failed models hidden; learning curve by model version |

**Checks** (all nine ran; every problem they found was fixed in two fix rounds):

| Check | Result |
|---|---|
| 1-2. Test suites, new tests per feature | 213 passed, 1 skipped (slow training test; passed when run on its own) |
| 3. Generic wording | A test fails the build if "Alvik" shows outside the Alvik set; "in real life" appears nowhere |
| 4. Browser click-through | 2 blockers (Train crashed on a save race; the no-model baseline scored 0% for non-Alvik projects) and 6 should-fix items: all fixed |
| 5. Clean install | About 14 minutes and several GB on a slow Linux link; launchers now find Python on Windows (`py -3`, then `python`); README gives real figures; fully offline after install |
| 6. An old project opens unchanged | Passed at every stage: no label lost, moved, retyped or restatused |
| 7. Newcomer (a student with 3D-printed shapes) | Got stuck on training (it stopped after one round and saved a model that found nothing) and on making a recording without the Pi: both fixed (training no longer stops while it finds nothing; any video file can be added) |
| 8. Code review | 8 findings, 2 that could put wrong labels into a project (same-picture accept copied a heading onto turned robots; undo auto-accept could delete change-all fixes): all fixed and re-tested with the reviewer's scripts |
| 9. Verifier | This report checked against the repository (below) |

**Conclusion.** The handoff version is complete and checked. A real training run through the studio
on a synthetic 5-minute recording with 40 spread-out labels found every object on held-back frames
(the old code would have stopped at round 13 with an empty model).

**What's next.**
1. Nick pulls the build onto his laptop and finishes his first 100 frames, then trains his first
   model.
2. The second stage, after Thursday's usage reset: smarter tracking (the chain, split branches), labels
   from robot logs, docks, live roll call, the AI helper, the extras.
3. The handbook's recognition section is updated from the README (not done in this build).

**Cost.** Subagent tokens about 2.8M (polish 0.35M, use-it 0.16M, share 0.09M, investigate and stats
0.37M, checks 0.94M, fix rounds 0.78M, fixture 0.09M), plus the main session's (not counted). Meter
before: 66% (Nick, 13:55 ET). After: to be read by Nick.

## New tests that may now be needed
- Windows: double-click both launchers on a clean Windows machine (not possible here), including a
  user name with a space and a OneDrive-synced Documents folder.
- Real footage: investigate's scan speed and verdicts on the 16-minute roam1 recording; the
  same-picture patch test (cutoff 2.5) on real robots turning slowly.
- Training on Nick's real labels: whether 30-second blocks over 16 minutes give each part enough
  frames, and the first model's score.
- The second-camera referee and the background second-camera pass on real two-camera footage.
- `use_model.py` on the Pi's live stream and a USB camera.

## Possible inaccuracies in what this relied on
- Every check ran on synthetic recordings; none on real footage.
- Chosen, not derived: the same-picture cutoff 2.5 (from measured synthetic turns), the 10 s
  same-picture window, investigate's spot size (0.7 of an object), 480 sampled frames per scan,
  timing caps (pauses over 20 s count as 20 s, 120 s per frame), the 15-round training minimum,
  the 30 s block length once there are 6+ minutes of footage, the 960 px limit for imported videos,
  the green/amber/red confidence thresholds.
- Install time and size were measured on Linux with a GPU build of PyTorch; Windows uses a smaller
  CPU build, so its figures are estimates.
- Time-saved figures for frames finished before this build are estimates from save times.
