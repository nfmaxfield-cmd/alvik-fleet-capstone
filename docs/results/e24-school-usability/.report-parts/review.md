What I checked after building, and what I did about it.

- **The acceptance runs found a fault that was not in either report.** In one gyro-jump run on the new code the
  start's 3 cm crawl read Bravo's tag as turned +67 degrees (it is at +90), so every heading of Bravo was 23 degrees
  out: 5 of 8 points, and its middle touched a keep-out. The crawl is e15's code; the stand-in's camera losses exposed
  it. Fixed: an answer between two quarter turns is crawled once more and averaged (`MIRLT_CRAWL_AGAIN=0`: as before),
  with a test. That first run is kept (`acceptance/first_yaw_s7.json`) and the seed was run again on the final code.
- **The plain 4 + 4 list shows no gain**, because the code before was already clean on it in two of three seeds. The
  gains are in the cases with a fault in them (restart, gyro jump, a missing robot) and in the endings' wording.
- **Low battery costs time by design.** With Bravo sent home, Alpha drives all 12 points: about 330 s against 118 s
  before. That is Q-b working as built, and a reason to confirm Q-b.
- **Before's "done" was wrong in 7 of 17 runs** (points missed, ending "done"). After, every ending matched the
  points truly reached.
- **Mid-batch code change.** The crawl fix went in while the batch ran. The after-runs made before it (4 + 4, gyro
  jump seeds 5 and 6, restart seed 5) did not meet the condition it changes (their crawls snapped to a quarter turn),
  so they were not run again.
- **What I could not check here:** anything on the Pi itself (the update on the real Pi, `space`'s total, the capture
  rate rule on live load, the phone in a hand).
