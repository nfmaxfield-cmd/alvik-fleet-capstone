
![Points reached and run time for every acceptance run, before and after](../results/e24-school-usability/acceptance_runs.png)

Each bar is one simulator run: on the code before, 9 of 17 runs missed points and 7 of those 9 still ended "done"; on this build all 17 reached every point, in 320 of 320 points against 261 of 320.

What the rows show, case by case:

- **4 + 4:** the same on both (the code before was already clean on it in two of three seeds).
- **20 + 20:** before, one seed in three ended in the thrash (8 of 40, stopped by the runner at 19 minutes after 35 re-plans). After: 40 of 40 three times, no re-plan.
- **Crossed lists:** before 38 of 40 twice, said "done". After 40 of 40 twice, 200 s quicker.
- **Restart:** before 9 and 15 of 16. After 16 of 16 twice.
- **Gyro jump:** before 4, 5 and 8 of 8, said "done" each time, and the robot's middle came within 0.0, 1.7 and 5.4 cm of a keep-out (under 4.75 cm the body is over it). After: 8 of 8 three times, the jump found and the turn driven again, 12.7 cm from the keep-out. One after-run failed first for another reason (Appendix D).
- **Low battery:** all points both ways. After takes about 330 s against 118 s, because Bravo goes home and Alpha drives all 12 (Q-b).
- **One robot missing:** before, the missing robot's four points were never driven and the run said "done". After, the robot that is there drives all 8.
