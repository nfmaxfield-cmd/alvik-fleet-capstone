# alvik-fleet-capstone

ISE 401 capstone (SWARM project): making a fleet of Arduino Alvik robots on the
8×8 tape-grid testbed smarter and faster.

This builds on Andrew Muszynski's work. His repositories are the reference
implementation:

| Repo | What it is |
| --- | --- |
| [AGV-Line-Following-Factory](https://github.com/Andrew-Muszynski/AGV-Line-Following-Factory/tree/VLM-integration) (`VLM-integration` branch) | Current fleet stack: AprilTag localization, micro-ROS firmware, navigator, dashboard |
| [Alvik-Factory](https://github.com/Andrew-Muszynski/Alvik-Factory) | Earlier ROS 2 / micro-ROS line-following AGV |
| [SwarmSLAM](https://github.com/Andrew-Muszynski/SwarmSLAM) | Earlier overhead-camera swarm with a Flask server |

Andrew's code is not copied here. It will only be added with his permission, since
his repos carry no license.

## Goal

How much does adding sensors improve an Alvik's locational accuracy? A series
of weekly experiments (E1-E8) measures the robot on odometry alone, the
overhead AprilTag camera, and the two combined, against the proposal target of
under 5 cm. The running record is
[docs/experiments/INDEX.md](docs/experiments/INDEX.md).

Earlier candidate areas, from the onboarding review:

1. Drive through straight grid nodes instead of stopping at each one
2. Faster, confirmed in-place turns
3. Multi-agent path finding in place of delay-only conflict avoidance
4. One planning model fitted to measured travel times
5. Depot throughput (single entry/exit edge)

## Layout

```
firmware/
  sensor_explorer/   print every sensor reading over USB
  home_lab/          WiFi-logged experiments: sensor stream, turn test,
                     lap timer, grid test, demo routes, straight-line test
tools/
  alvik_console.py   laptop console: saves robot logs to CSV, sends commands
  summarize.py       tables and stats from saved sessions
  make_run_order.py  shuffled run order + tape measurement sheet per experiment
  tape_results.py    tape measurements -> errors, stats, chart for a record
docs/
  home-experiments.md  how to run the home experiments
  experiments/         one record per experiment (E0-E8), index, template
```

Start with [docs/home-experiments.md](docs/home-experiments.md).

## Secrets

This repo is public. WiFi credentials, agent IPs and other lab details go in
files that are git-ignored:

- `firmware/home_lab/secrets.h`: copy `secrets.example.h` in that folder and fill it in
- `.env`: for Python tools

Never commit real credentials.
