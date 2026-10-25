# Mark the robot (MIRLT): hand-checked positions from the laptop

**What it's for.** The tracker experiments (docs/experiments/DOE-tracking.md, DOE 1, and the "Camera and tracking"
rows of docs/doe-backlog.md) need the robot's true position about once a second in recorded footage. Instead of
measuring on the floor, you mark it on the pictures: the page shows each picture with a green ring where the tracker
thinks the robot is, you slide the ring onto the robot and press Confirm. The program saves the mark in pixels and
in floor cm.

**First job: the three 2026-10-22 evening stretches.** Left 19:30, Center 20:09 and Right 20:26: 6 minutes of
footage, about 370 pictures. Expect about 15 to 25 minutes in all (most pictures need only Enter). You can stop and
carry on later at any point.

## 1. Get the newest code (once)

1. Open **File Explorer** and go into the project folder **alvik-fleet-capstone**.
2. Click the **address bar** at the top of the File Explorer window (where the folder path is), type `powershell`
   and press **Enter**. A blue PowerShell window opens, already in the project folder.
3. In the PowerShell window, type this and press **Enter**:

```
git pull
```

You should see a list of changed files ending with a line like `... files changed`, or `Already up to date.`

## 2. Start the page

In the project folder (File Explorer), **double-click `mark-the-robot.bat`**.

- A black window opens. The first time, it may install two Python packages (a minute of text scrolling).
- It then says `Mark the robot is running. Open http://localhost:8770 ...`, and your browser opens a dark page
  titled **Which stretch?** with three cards.
- **Leave the black window open** while you mark (it is the program). Closing it stops the page; your marks are
  already saved.

If the browser doesn't open by itself: open Chrome or Edge, click the address bar, type `localhost:8770` and press
**Enter**.

## 3. Pick a stretch (this gets the footage onto the laptop)

Click **Download and mark** on the first card (Left). The page downloads only that stretch from GitHub, about
100 MB (the black window shows `35% ... 70% ... 100%`). It takes a minute or two on home WiFi. The footage is saved
in `Documents\mirlt-footage\`, so a second start doesn't download it again.

Then the page shows the first picture with a **green ring**, and a progress bar along the top.

## 4. Mark each picture

| What you see | What to do |
| --- | --- |
| The green ring sits on the robot | Press **Enter** (or the green **Confirm** button) |
| The ring is a bit off | **Drag anywhere** on the picture: the ring slides with your mouse. Or **click** on the robot to put the ring there. Then **Enter** |
| The robot is hidden (a hand over it) or out of the picture | **N** (Not visible) |
| Several robots close together and you can't tell which is the one named at the top | Put the ring on your best guess, then **W** (Not sure which) |
| You'd rather not judge this one | **S** (Skip) |
| You want to see which robot was moving | **P** plays the second before this picture |
| You pressed the wrong thing | **Z** (Undo) takes back the last mark |

- **Where to put the ring:** on the robot itself, centred on its body. Not on the bright glow its lights throw on
  the floor beside it. The ring is a robot's size, so a good mark has the ring around the robot's top edge evenly.
- **Arrow keys** nudge the ring by one pixel (hold **Shift** for 5). One pixel is about 0.4 cm on the board.
- **Zoom:** the picture is zoomed in on the ring already. The buttons on the right (Whole, 2x ... 6x) change it,
  and the small picture under them shows the whole camera view: click it to jump the ring there.
- **The dashed grey ring** is your mark on the picture before (the robot moves at most a few cm in a second).
- When you move a ring, the tracker starts again from your mark, so the next pictures' rings follow the robot.
  For a second or two after a correction, the line above the picture may say "The tracker is still working"; the
  ring then sits on your last mark and jumps to the tracker's guess when it is ready (unless you already moved it).
- At the end, a few pictures come back with the ring put **off on purpose** ("Repeat check" at the top). Mark them
  like the others: they measure how exactly marks repeat.

**When it says "All marked"** (a green box on the right): click **Stretches** (top right) and do the next card.

## 5. Send the marks

The green "All marked" box shows the folder the marks are in:
`alvik-fleet-capstone\tools\experiment\data\marks\<stretch name>\`. Open it in File Explorer and drag
**marks.csv** and **summary.json** into the chat (for each of the three stretches).

## On a phone instead

The page works on a phone too (drag with your finger; the buttons sit along the bottom). In the PowerShell window
from step 1, type this and press **Enter**:

```
py -3 tools\label\mark_robot.py --phone
```

The window prints a line `On a phone on the same WiFi: http://192.168.x.x:8770`. Type that address into the
phone's browser. Keep the laptop on and the window open while you mark.

## Other footage: troubleshooting a session

To check a moment where the runner lost a robot in any session uploaded to GitHub (its release name is in the
runner's last message, `All up: ...e02-session-<date-time>`). In the PowerShell window, one line (change the
release name, robot and times):

```
py -3 tools\label\mark_robot.py --release e02-session-20261024-193526 --robot Right --from 19:40:00 --to 19:42:00
```

It downloads only those two minutes and the session's logs. Each picture then also shows a **red cross**: where the
runner thought the robot was. The saved marks include how far that was from your mark (the summary's `runner` line).

For a recording folder already on the laptop (copied from the Pi), give the folder instead of `--release`:

```
py -3 tools\label\mark_robot.py "C:\Users\Nick\Desktop\recordings\20261025-101500_E02" --robot Left --from 10:20:05 --to 10:22:00
```

## If something goes wrong

| What you see | What to do |
| --- | --- |
| `Python not found` in the black window | Install Python from python.org (tick **Add python.exe to PATH**), then double-click the .bat again |
| `Ports 8770-8779 are all busy` | Another mark-the-robot window is still open: close it, then double-click again |
| The page says "The program is not answering" | The black window was closed: double-click the .bat again (your marks are kept) |
| The download stops with an error | Check the WiFi, then click the card again (it starts the download over) |
| "No tracker guesses" under **So far** | The floor calibration file is missing: `git pull` (step 1). You can still mark; the ring follows your last mark |
