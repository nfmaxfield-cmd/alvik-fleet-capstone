# Session timeout: an unattended session always ends and uploads (MIRLT)

**Premise.** Nick asked for this at 08:44 on 2026-10-26. Last night's session (`--hours 1.5`, started 21:14) reached
its planned end around 22:45. Then a hold or question sat unanswered all night. Nothing uploaded until Nick stopped it
at 06:40, and the recorder filled 23 GB of video. Built and tested in the simulator only; the Pi was not touched.

**What changed** (`tools/experiment/e02_timeout.py`, new; `e02.py`; `e02_view.py`; one note in `e02_analysis.py`):

| Rule | Default | Option (0 = off) |
|---|---|---|
| After the planned end (launch + `--hours`), a hold or question unanswered this long ends the session | 10 min | `--after-end-timeout-min` |
| At any time, a hold or question unanswered this long ends it ("ended: no answer for 30 min") | 30 min | `--answer-timeout-min` |
| Hard cap at planned end + this: no new run starts; the current run finishes and parks if it does so within 180 s, else every robot is just stopped | 30 min | `--hard-cap-min` |

- **Ending** means every robot is stopped, the state is saved, and the runner exits normally (exit code 0). The Pi's
  runner window then runs `upload_session.py --stop-recorder` as it always did.
- The recorder check's pause ("the Pi recorder is not answering") waits on Nick like a hold, so the same limits apply.
- A question that already takes its default after a time limit keeps it (the live page's 15 min, `--help-wait-min`).
  Only the hard cap can come before its default.
- The live page shows the deadline under a hold or question: "No answer by 23:05 ends the session and uploads". The
  hold panel no longer says "no time limit". The console says the same.
- The reason the session ended is in `e02_log.txt` ("Session ended: ..."), in `e02_state.json` (`session_end`: key,
  words, when asked, what it waited on; `session_timeouts`: the settings per launch), and as the first note of the
  analysis `report.md` when it was a timeout. A timed-out hold's answer is recorded as `timeout` in `holds`.
- **Backstop:** if the runner has not finished 90 s after a timeout, it is interrupted like Ctrl+C.
- DOE notes are in the code. All five numbers are chosen convenience settings, so no DOE is needed.

**Sources.** 4 dry runs, 22 new unit tests (`tests/test_e02_timeout.py`). The whole suite passes: 479 passed,
18 skipped (`tools/experiment/tests` and `tools/pi/tests`).

## Data

All runs: `e02.py --dry-run --auto --home --skip-naming --skip-find-check` plus the arguments below. A
`--dry-scene` answer of `none` (new) leaves that hold unanswered on the live page. Timeouts were shortened to 0.5-4 min
so each run ends in minutes.

| Run | Set-up | Hold put to Nick | Ended | Reason logged | Exit |
|---|---|---|---|---|---|
| A | `--hours 0.01 --after-end-timeout-min 0.5`, hold at Right's 2nd ray (after the planned end, 08:55) | 09:02:46 | 09:03:16 (30 s) | no answer for 0.5 min after the planned end (08:55) | 0, at 09:03:21 |
| B | `--hours 1 --answer-timeout-min 0.5`, same hold (during the session) | 08:58:38 | 08:59:08 (30 s) | no answer for 0.5 min | 0, at 08:59:13 |
| C | `--hours 0.01 --hard-cap-min 1`, no hold; cap lands in explore | cap 08:57:01 | 09:00:01 (after the 180 s grace) | hard cap reached, planned time + 1 min | 0, at 09:00:04 |
| D | `--hours 0.01 --hard-cap-min 4 --classic-setup --dry-true-calib`; cap lands in run 2 | cap 09:04:35 | run 2 finished and parked 09:05:30; no new run | hard cap reached, planned time + 4 min | 0, at 09:05:32 |

**Run A inside a copy of the Pi's runner window.** Run A ran in a tmux session called `e02`, set up as
`run-e02-pi.sh` sets it up. A stand-in recorder ran in window `rec`. The window `run` ran the same chain: `e02.py`,
then `;`, then `upload_session.py --recordings ... --stop-recorder`. Only the network was stubbed: a stand-in token,
and every GitHub request recorded instead of sent. After the timeout:
- the runner exited (09:03:21);
- the recorder got Ctrl+C and saved (09:03:21);
- the upload made its release and sent `logs-e02-...-dryrun.zip` (14.7 MB) and the recording's small-files zip.

The live page showed "No answer by 09:03 ends the session" under the hold for the 30 s it was open; status.json was
read every 2 s.

## Conclusions

1. A hold nobody answers now ends the session: 30 min during the session, 10 min after the planned end. Runs A and B
   ended at exactly 30 s, their shortened limit.
2. The hard cap wraps up as asked. In the runs (D), the current run finished and parked, and no new one started. In
   setup (C), there was no run to finish, so every robot was stopped after the 180 s grace.
3. `run-e02-pi.sh` needs no change. Its runner window joins the runner and the upload with `;`, so the upload and the
   recorder stop run after any end. A timeout end exits normally (code 0) and promptly (2-5 s after the decision).
4. Last night under these rules: planned end about 22:44. Any hold left open then would have ended the session by
   about 22:54, and a hold from the last half hour by 23:14 at the latest (the cap). The upload would then have started.

## What's next

1. The next real session on the Pi is the real check. After it, look in `e02_state.json` for `session_end`.
2. The other agent's analysis of last night's session (`analyses_oct25/session_2114`) should say what the open hold or
   question was. If it was something that never reaches `ask()` or a hold, tell me: the hard cap still ends it, but
   only at planned + 30 min.

**Questions for Nick**

*Substantive*
1. After the planned end, a setup-style question that takes its default after 15 min keeps its 15 min. The 10-minute
   rule does not shorten it. Its default lets the session carry on, so it cannot strand a session. My assumption: that
   is what "keep it" meant. If you want 10 minutes for everything after the planned end, it is a one-line change.
2. The planned end counts from when the runner starts, setup included. That matches last night (21:14 + 1.5 h ≈
   22:45). A `--resume` launch counts from its own start. Is that right, or should a resumed session keep the
   original end?
3. The hard cap also applies when you are at home watching. A session that runs long on purpose needs a bigger
   `--hard-cap-min`, or 0 to switch it off. Should home sessions (`--home`) get no cap by default?

*Other*

4. The 180 s grace for the current run to finish and park is my choice. A run with parking takes about 70 s, and
   run D's took 55 s after the cap. A DOE is not needed for it; it is a convenience setting.

## Sweep 1: new tests that may now be needed

- A real Pi session ending by timeout: the upload and the recorder stop with the real token and the real camrec.py.
  The dry run stubbed GitHub and the recorder.
- The 90 s backstop (Ctrl+C to itself) was only unit-tested with a stand-in. No dry run got stuck long enough to
  need it.
- A laptop session (`run-e02.bat`, console questions, no live page): the console timeout is unit-tested only.
- A `--plan` run: each stage now has its own hard cap (`--hours` + 30 min). Stage 4 (`--hours 4`, "until the
  batteries run out") will now stop at 4.5 h.

## Sweep 2: possible inaccuracies in what this relied on

- 30 / 10 / 30 min are Nick's numbers. 180 s grace and 90 s backstop are chosen, not derived.
- What actually waited all night is assumed to be a hold or an `ask()` question; the analysis of that session was
  not finished when this was built.
- The page shows the deadline to the minute, rounded down. The real end comes up to 59 s after the time shown.
- The dry runs used shortened limits (0.5-4 min). The full-length numbers are covered only by unit tests on the clock.
- The planned end of 22:44-22:45 last night is from Nick's start time and `--hours`. Neither log nor state was read
  here.
