# Standing rules for work in this repo

- **Screenshots (Nick, 2026-10-26):** every build saves screenshots of the features it adds and the problems it
  finds or fixes (live page and live lab via Playwright on dry/simulator runs, charts, terminal output where it
  helps), so Nick can show them to people. Save to `docs/screenshots/<YYYY-MM-DD>/` with descriptive file names and
  add one line per picture to `docs/screenshots/index.md`: what it shows, **feature** or **problem**, and the build
  or report it belongs to. Append to the index; never overwrite another build's lines.
- **Heat tracker (Nick, 2026-10-26):** every real session's analysis runs `python3 tools/analysis/heat_tracker.py
  <session folder> [--cooling=<what cooling the Pi had>]`, which adds the session's row and chart to
  `docs/results/heat-tracker.md`, plus a line under "Readings" saying what it shows.
