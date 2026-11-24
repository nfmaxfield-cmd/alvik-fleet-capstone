- **On the school table (first session after the update):** a 4 + 4 and a 20 + 20 run; a deliberate Stop during
  start-up; reload the page mid-run, then Run again; a second Ctrl+C during the wrap-up of a terminal run.
- **Heading check on real pictures:** how often it waits and for how long (`turn_heading` in the summary), and whether
  any "corrected" turn was a real jump (compare with the raw gyro rows). A false correction on real lag is the risk.
- **Capture rate rule on the Pi:** one session at the automatic rate with the heat tracker, to see what it picks with
  two robots and whether time unseen changes.
- **The update on the real Pi**, including a second run of `./run-fleet-pi.sh update` straight after (should say
  "already on GitHub's newest code").
- **Page tests in the suite.** The page's behaviour is checked by Playwright scripts kept in the results folder, not
  by pytest. A small pytest wrapper that runs them against practice mode would catch a broken page before a push.
- **N5 wording** (a heading disagreement said in degrees) has no test of its own.
- **Crawl again:** a simulator test that makes the first crawl read badly end to end (the unit test stubs the crawl).
- **A single robot under the battery floor** (nobody to hand over to): only reasoned, not run.
- **Naming with three or more robots** and late pictures (tested with two).
