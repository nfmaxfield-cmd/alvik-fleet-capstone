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
| Several robots close together and you can't tell which is the one named at the top | Put the ring on your best guess, then **W** (Not sure which). It is kept, but left out of the truth the experiments score against |
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

## 5. The marks upload themselves

Once every picture of a stretch is marked, the page puts the marks on GitHub by itself (the **Upload to GitHub** box
on the right says `Uploaded at 14:05`). If you change a mark afterwards, it uploads again a few seconds later. The
**Upload now** button sends what there is at any time (half-done is fine). Nothing to drag into the chat.

This needs a GitHub token on the laptop, once (part 6). Without one, the box says `No GitHub token on this laptop
yet`: the marks stay saved on the laptop, in `alvik-fleet-capstone\tools\experiment\data\marks\<stretch name>\`.

## 6. GitHub token (once, about 5 minutes)

A token is a password that lets the laptop put files on the project's GitHub page, and nothing else. Make a **new**
one for the laptop: the Pi keeps its own (regenerating the Pi's token would stop the Pi's uploads until you save
the new one there too).

**Make the token** (in Chrome or Edge, signed in to GitHub as nfmaxfield-cmd):

1. Click the address bar, type this and press **Enter**:

```
github.com/settings/personal-access-tokens/new
```

2. You see **New fine-grained personal access token**. Fill in:
   - **Token name:** `mirlt-laptop`
   - **Expiration:** click it and pick the longest it offers (1 year, or No expiration if it is there).
   - **Repository access:** click **Only select repositories**, then the **Select repositories** list, and click
     **alvik-fleet-capstone**.
   - **Permissions:** under **Repository permissions**, find **Contents** (or click **Add permissions** and tick
     **Contents**) and set it to **Read and write**. GitHub adds **Metadata: Read-only** by itself; leave it.
3. Click **Generate token** at the bottom (and **Generate token** again if it asks to confirm).
4. The token shows once, starting `github_pat_`. Click the **copy** icon next to it. Leave this page open until
   step 6 below says `Saved`.

**Save it on the laptop:**

5. Open **File Explorer**, go into the project folder **alvik-fleet-capstone**, click the **address bar**, type
   `powershell` and press **Enter**. A blue PowerShell window opens. Type this and press **Enter**:

```
py -3 tools\pi\upload_session.py --set-token
```

6. It says `Paste the GitHub token (it will not show) and press Enter:`. **Right-click** once inside the blue window
   (that pastes; nothing appears, on purpose), then press **Enter**. You should see:
   `Saved. This computer can upload to nfmaxfield-cmd/alvik-fleet-capstone now (saved in C:\Users\...\github_token).`

**Check it works** (any time; in the same blue window):

```
py -3 tools\pi\upload_session.py --check-token
```

You should see `Token works: this computer can upload to nfmaxfield-cmd/alvik-fleet-capstone.`

| What it says | What to do |
| --- | --- |
| `GitHub refused that token (401)` | It was not copied whole. Go back to the token page, make a new one (steps 1-4), paste again |
| `... can read the repository but not write to it` | **Contents** was left at Read-only. Make a new token with **Read and write** |
| `GitHub refused the saved token (401)` (the check) | It expired. Open `github.com/settings/personal-access-tokens`, click **mirlt-laptop**, click **Regenerate token**, pick the expiration, click **Regenerate token**, copy it, then do steps 5 and 6 again |
| `Could not reach GitHub` | Check the WiFi and run the check again |

## 7. Size check: is the camera's scale right? (optional, about 10 minutes)

The floor calibration turns pixels into cm. The robot's top is 9.60 cm square, so measured through the calibration
it should come out 9.60 cm everywhere on the board. If it comes out bigger or smaller, the calibration's scale is off
there.

1. On any marked picture, click **Fit the size here** (in the **Size check** box on the right, or press **F**).
   Four small blue squares appear on the robot's corners, with each side's length in cm.
2. Drag each square onto the corner of the robot's **white shell top** (not its shadow or the glow on the floor).
   The picture zooms in to 6x for this. **Tab** picks the next corner; the arrow keys move it a quarter pixel.
3. Press **Enter** (or **Save fit**). The box shows `This outline: 9.71 x 9.64 cm ... +0.9 %`.
4. Click **Next picture in an unchecked part**: it opens a marked picture in the part of the board with the fewest
   fits. Repeat until the 3 x 3 grid has 2 or 3 fits in each square (about 15 in all).
5. The grid shows each part of the board: green = right to within 2 %, red = off, amber = not enough fits yet.
6. Click **Measure every marked picture automatically**: the program finds the robot's outline by itself on every
   marked picture. The pictures you fitted by hand tell it how much shadow or glow it takes in, and it corrects for
   that.

Once the dock is printed, choose **the dock's marker strip** in the list and fit its four corners the same way
(measure the printed strip with calipers first; start the page with `--strip-mm 90.1` if it is not 90.0 mm).

The fits are saved with the marks (`size_fits.csv`) and go up with them.

## 8. From the live view: mark this moment

During a session, the live view (`localhost:8090`) has a **Mark this moment** button under the camera picture.
Click it when the runner seems to have lost a robot. A new tab opens with the mark page on the **last 20 seconds**
of pictures of the robot being followed: the ring starts where the runner thought the robot was (the red cross),
you put it where the robot really is. The session carries on meanwhile. The pictures and marks are saved in the
session folder, `logs\e02-<date-time>\moments\`, and the marks upload themselves like the others.

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
| The **Upload to GitHub** box says `No GitHub token` | Part 6 |
| It says `The upload did not work` | Check the WiFi, then click **Upload now** |
