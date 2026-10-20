# Recognition studio: list for the next build

A running list, kept while using the studio on real footage. Each item says who raised it.
Nothing here is built yet; the next build works through it.

## Name and branding (Nick)

- Rename the software **Maxfield IRL Tracking**, with "Identify. Recognize. Label." written
  in smaller letters underneath (what IRL stands for). Applies to the page title, the top bar,
  the launcher window, the report, the README and the handbook section.

## Feels generic, not Alvik-specific (Nick)

The studio should not say "Alvik" until the project is about Alviks. Places it still does
(found 2026-10-20):

- Page title and top bar say "Alvik labelling" (`tools/label/page.html`).
- New projects start with the object types alvik, hand, unknown, machine
  (`tools/recognition/project.py` DEFAULT_CLASSES, `tools/label/app.py` CLASSES). Proposal: a
  new project starts with only "unknown", and the first screen asks what objects you track,
  with "Alvik robots" offered as a ready-made set.
- Colours are fixed per Alvik type name (`--alvik`, `--hand`, ...); give types colours from a
  palette by position instead.
- README opens "Built for the Alvik robots".

## Found while using it (Claude)

- After **Add recording** the page stayed blank instead of showing the progress bar
  (fixed 2026-10-20, commit after 8e1bcc5; reaches the laptop on the next pull).
- The black window's time estimate for "finding moving objects" was far too high (said
  ~1000 s left; it took about 12 minutes for 24,000 frames). Estimate from the recent rate,
  not the first frames.
- Adding a two-camera session opens (and runs the motion pass on) the first camera only; the
  second camera's pass should start when you first switch to it (to confirm on roam1). Consider running both in the background, so the
  second is ready when you get there.
