| Suite | Result on the final code |
|---|---|
| Fleet (`tools/fleet/tests`) | 1239 passed, 10 skipped, 1 failed once (below), in 30 min 47 s |
| Pi (`tools/pi/tests`) | 140 passed |
| Analysis (`tools/analysis`) | 16 passed |
| Simulator (`tools/sim/tests`) | 122 passed, 2 failed: the two in `test_follow.py` that failed before this build |

The one fleet failure is `test_e22_no_standing.py::test_a_standing_robot_31_cm_off_is_corrected...` (a free roam test;
4.3 cm against a 4.0 cm limit). It passed 5 of 5 times alone, 2 of 2 with its whole file and once with its neighbour
files, and this build does not touch the free roam controller. I did not find why it failed once in the full run; it is
in Sweep 1.

New tests: 24 in `tools/fleet/tests/test_e24_school_usability.py` and 3 in `tools/pi/tests/test_update_pi.py`. Run
against `39df4cf`: 24 fail, 2 are skipped (there is no update command there to test) and 1 passes (it tests the
simulator's new gyro-jump injection, a simulator-only file that had to be copied to the old code for the acceptance
runs). The fleet and Pi suites were run before the last simulator-only and results-folder commits; the crawl fix
(`MIRLT_CRAWL_AGAIN`) and its test were in.
