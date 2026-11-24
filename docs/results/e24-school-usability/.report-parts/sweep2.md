- **The stand-in is a model of 23 Nov, not the Pi.** Loss lengths are drawn from that day's 956 measurements and
  placed at random; real losses came in bursts tied to the Pi's load. The planning factor (15.1) was fitted to the
  planner before this build; the quicker planner may not scale the same way on the Pi. Re-fit after the next session.
- **Two or three seeds per case.** The ranges say what happened in those runs. A difference of one point in one run
  is not evidence either way.
- **The simulator's gyro jump** is one step of 25 degrees at the start of a turn, as measured once. Real jumps may
  come mid-leg; the raw-row watch covers that case only in a unit test.
- **18 Nov run 14** was read from its raw rows by me alone; the conclusion (not a heading fault) contradicts the
  23 Nov report's candidate cause and should be looked at by a second pair of eyes.
- **"Same plans step for step"** was checked on 17 saved re-plans and the two first plans, not on every possible map.
  The old way stays behind `MIRLT_PLAN_FAST=0`.
- **Mocked screenshots.** States that need real robots or a fault (dead server, no pictures, the real start panel)
  are the real page fed staged answers. The server side of those states is covered by unit tests, not by a real fault.
- **Chosen numbers** are in Appendix A with a DOE yes or no for each. One is a yes: the capture rate's "at most 2 x
  what the bridge reads".
- **`space` on the Pi** (B12): the fix was made from the code; the sandbox's disk is not the Pi's card.
- **Time planning costs on the Pi** in section 3 is this machine's time multiplied by 15.1, not a measurement.
