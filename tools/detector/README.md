# Camera detector (tools/detector)

Finds robots (and hands, unknown objects, machines) in the overhead camera
picture and reports, for each one, its class, centre in image pixels,
heading, and a confidence score. Built for the project, not taken from YOLO.

## What the model is

A small network of our own design (`model.py`, class `CentreNet`). It looks
at the camera frame shrunk to 320x240 and, for every class, paints a
"heat map" a quarter of that size that is bright where it thinks an object's
**centre** is. For each spot on that map it also guesses:

- **offset**: where exactly inside the 4x4-pixel block the centre lies (so
  positions are not rounded to the block grid)
- **heading**: a second small heat map that is bright at each robot's
  **front point** (a spot about a third of a robot ahead of its centre, where
  the Alvik's LEDs are), with its own offset. The heading is the direction
  from the centre to the front point found at about the right distance. If
  no front point is found, the heading is reported as unknown (`None`)
  rather than guessed. Only classes with a front get a heading (alvik and
  machine by default).

Finding objects is then "find the bright spots above a threshold" (a spot
counts if it is the brightest in its 3x3 neighbourhood).

Why a front point and not "predict the angle": the first version predicted
sin and cos of the heading at the centre (as CenterNet does for car
orientation). On synthetic data that stayed at chance level (error ~90 deg)
after 500 training steps, while the front-point version was at ~40 deg after
240 steps and kept improving: spotting "the front" is a local pattern job,
like spotting centres, which small networks learn quickly. The sin/cos
version is still available (`train.py --heading-mode sincos`) for comparison
on real data. The idea comes from
CenterNet ("Objects as Points", Zhou et al. 2019); the layers are our own
compact design (MobileNetV2-style blocks with a small feature pyramid).

Why this shape of model: the robots are small (about 30 px), all the same
size, seen from straight above, and what we need is a **position and
heading**, not a box. A centre map gives exactly that, with no box maths, and
stays cheap enough for the Pi.

It is multi-class from the start: the class list is a setting
(`alvik, hand, unknown, machine` by default), one heat map per class, and
classes can be appended later (see "Adding a class").

Speed vs accuracy is set by `--width` (channels) and `--depth` (blocks per
stage) and the input size. The default (width 1.0, depth 1, 320x240,
colour) has about 77k parameters and 99 M multiply-adds per frame.

## Files

| File | What it does |
| --- | --- |
| `detcore.py` | shared numpy/OpenCV code: reading labels, split by recording, pre-processing, decoding, scoring, homography |
| `model.py` | the network; `python model.py` prints sizes of a few designs |
| `dataset.py` | training frames, augmentation, training targets (PyTorch) |
| `train.py` | training loop (CPU or GPU), early stopping, log CSV |
| `export.py` | `.pt` -> `.onnx` (optionally an 8-bit `.int8.onnx`) |
| `infer.py` | runs the `.onnx` with ONNX Runtime: `load()` / `predict()`; no PyTorch needed (Pi) |
| `predict_torch.py` | the same interface on the `.pt` weights (laptop) |
| `benchmark.py` | time per frame and frames per second, per size and thread count |
| `evaluate.py` | precision / recall / F1, centre and heading errors, per class; compares detectors |
| `baseline_tv.py` | off-the-shelf comparison: torchvision SSDlite / Faster R-CNN fine-tuned on our labels |
| `baseline_yolo.py` | **research only, not shipped**: Ultralytics YOLO (AGPL-3.0) fine-tuned on our labels, for the report's comparison |
| `make_synthetic.py` | fake labelled recordings, for testing the pipeline without real labels |
| `tests/` | `python -m pytest tools/detector/tests` |

## Data it trains on

Labels folders as in `docs/data-formats.md` ("Labels for the camera
detector"): `<recording>/labels/labels.csv` and `frames/<frame:06d>.jpg`.
Rules used when reading:

- rows with status `confirmed` or `corrected` are the truth; `rejected` rows are ignored
- a frame that still has a `pending` row is skipped completely (its unreviewed
  objects would otherwise teach the model "nothing here")
- a `none` row makes the frame a confirmed empty frame (useful negatives)
- heading is used only where `heading_known` is 1

**The split is by recording, never by frame.** Neighbouring frames of one video
are nearly the same picture, so a random frame split would test on frames the
model has practically seen. `train.py` hands whole recordings to train / val /
test (about 70/15/15 % of frames, needs at least 3 recordings) and writes
`split.json`; pass that file to later runs and to `evaluate.py` so every
comparison uses the same test recordings. You can also name them:
`--val 20261020-141500_E1 --test 20261022-101000_E2`.

## Install

Laptop (Windows or Linux), Python 3.10+:

```
pip install torch torchvision onnx onnxruntime opencv-python numpy pytest
```

(If that is slow: `pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu`.)
Tested with Python 3.11, torch 2.14.1, torchvision 0.29.1, onnx 1.23.2,
onnxruntime 1.25.0, OpenCV 4.13, numpy 2.4.

## Train locally

```
cd tools/detector
python train.py --labels D:/recordings --out runs/v1
```

`--labels` takes labels folders, recording folders or a parent folder (it
finds every `labels.csv` below it). Useful options: `--epochs 60`,
`--size 320x240`, `--in-ch 1` (greyscale), `--width 0.5 --depth 1` (smaller
and faster), `--max-minutes 30`, `--seed 0 --deterministic` (repeatable),
`--split runs/v1/split.json` (same split as an earlier run).

Output in `runs/v1/`: `best.pt`, `last.pt`, `train_log.csv` (one row per
epoch: losses, validation precision / recall / F1, centre error, heading
error), `split.json`, `config.json`.

On the 2-core test machine one training step (16 frames) took 1.5-4 s, so a
pass over ~1100 frames took 2-4 minutes; a modern 4-8 core laptop should be
2-4x faster, and a GPU (Colab) much faster. `--steps-per-epoch 40` makes
epochs shorter (more frequent validation) without changing anything else.

## Train on Google Colab (free GPU)

Runtime > Change runtime type > T4 GPU. Upload the recordings' `labels`
folders to Google Drive (e.g. `MyDrive/recordings/<recording>/labels/`), then
run these cells:

```
!git clone https://github.com/nfmaxfield-cmd/alvik-fleet-capstone.git
%cd alvik-fleet-capstone/tools/detector
!pip install -q onnx onnxruntime
```
```
from google.colab import drive
drive.mount('/content/drive')
```
```
!python train.py --labels /content/drive/MyDrive/recordings --out /content/drive/MyDrive/runs/v1 --epochs 80 --batch 32
!python export.py /content/drive/MyDrive/runs/v1/best.pt
!python evaluate.py --labels /content/drive/MyDrive/recordings --split /content/drive/MyDrive/runs/v1/split.json --model infer:/content/drive/MyDrive/runs/v1/best.onnx
```

(Colab already has PyTorch, torchvision and OpenCV. The clone only works once
this folder is pushed to GitHub. Saving to Drive keeps the results if the
Colab session ends.)

## Export and run on the Raspberry Pi

On the laptop:

```
python export.py runs/v1/best.pt            # -> runs/v1/best.onnx (about 300 KB)
scp runs/v1/best.onnx st1maxfini@st1maxfini.local:~/
```

The `.onnx` file carries the class list, input size and threshold, so it is
the only file the Pi needs besides this folder. On the Pi (64-bit Raspberry Pi
OS), once:

```
sudo apt install -y python3-opencv
python3 -m venv --system-site-packages ~/detenv
~/detenv/bin/pip install onnxruntime numpy
```

(Raspberry Pi OS refuses `pip install` outside a virtual environment; the
`--system-site-packages` venv reuses the apt OpenCV. Elsewhere, plain
`pip install onnxruntime opencv-python numpy` is enough.)

Then:

```
cd ~/alvik-fleet-capstone/tools/detector
~/detenv/bin/python infer.py ~/best.onnx --camera 0 --seconds 20            # live, prints ms per frame
~/detenv/bin/python infer.py ~/best.onnx --video ~/recordings/<rec>/video.avi --csv preds.csv
```

From Python (the detector interface in `docs/data-formats.md`):

```python
import infer
model = infer.load("best.onnx", threads=4)        # score_thresh=..., input_size=(w, h) optional
dets = infer.predict(model, frame_bgr)            # [{'cls': 'alvik', 'x': 312.4, 'y': 198.0, 'heading_deg': 87.5, 'score': 0.93}, ...]
```

`heading_deg` follows the labels convention (0 = image right, counter-clockwise
on screen positive); it is `None` for hands and unknown objects, and for a
robot whose front point was not found. On the laptop,
`predict_torch.load("runs/v1/best.pt")` gives the same answers without
exporting (slower; if another program is using the CPU, pass `threads=1`:
PyTorch's threads waiting on each other made it 100x slower in one test here).

**8-bit model (optional, often faster on ARM):**
`python export.py runs/v1/best.pt --int8 D:/recordings` also writes
`best.int8.onnx`, calibrated on real frames. Benchmark it on the Pi and
evaluate it before trusting it.

## Benchmark (laptop or Pi)

```
python3 benchmark.py --model best.onnx --image some_frame.jpg
python3 benchmark.py --model best.onnx best.int8.onnx --sizes 320x240,640x480 --threads 1,2,4 --csv bench.csv
python3 benchmark.py --arch 0.5x1,1.0x1,1.0x2 --sizes 320x240     # untrained designs (needs PyTorch)
python3 benchmark.py --detector baseline_tv --weights runs/base/best.pt --threads 4
```

It warms up first (the first runs are slow), then times each frame and prints
mean, p50 (typical) and p95 (slow frame: 95 % are faster) in milliseconds, and
frames per second. `model_ms` is the network alone; the other columns are the
whole `predict()` call (shrink frame + network + find peaks), which is what the
live system pays. Use a real frame (`--image`) and, on the Pi, check for heat
throttling (`vcgencmd get_throttled` should print `0x0`).

## Evaluate and compare

```
python evaluate.py --labels D:/recordings --split runs/v1/split.json \
    --model infer:runs/v1/best.onnx \
    --model baseline_tv:runs/base/best.pt \
    --model csv:someone_elses_predictions.csv --sweep --out eval.csv
```

- `--model KIND:PATH`, repeatable: `infer` (.onnx), `predict_torch` (.pt),
  `baseline_tv`, `baseline_yolo` (research only), any `.py` file with `load`/`predict`, or `csv` (columns
  `recording,frame,cls,x_px,y_px,heading_deg,score`, as written by
  `infer.py --csv` or `--save-predictions`)
- by default it scores the **test** recordings of the split; `--all` scores
  every labelled frame (for new recordings never used in training)
- a prediction matches a label of the same class within `--radius` px (default
  8 px, about a quarter of a robot; per class with `--radius 8,hand=25`);
  most confident predictions claim first
- reported per class and overall: precision (share of predictions that are
  real), recall (share of real objects found), F1, centre error (mean and p95
  px), heading error (mean, median, p95, and flipped% = more than 135 deg
  off, i.e. front and back confused), heading_given% (of the robots found
  whose label has a heading, the share the detector gave a heading for),
  milliseconds per frame
- **cm errors:** put `floor_calibration.json` (a 3x3 matrix mapping floor cm
  to image px, e.g. `{"H_floor_to_image": [[...],[...],[...]]}`) in the
  recording folder, or pass `--calibration file.json`; evaluate inverts it and
  adds cm columns
- `--sweep` also prints precision / recall / F1 at score thresholds 0.1 to 0.8,
  to choose the threshold

## Comparison baseline (off-the-shelf)

`baseline_tv.py` fine-tunes a general-purpose detector from torchvision:
SSDlite320-MobileNetV3 (`--arch ssdlite`, default) or Faster R-CNN
MobileNetV3-320-FPN (`--arch frcnn`), starting from COCO-pretrained weights.
Each centre label becomes a fixed-size box (alvik 30 px, hand 70 px, ...;
`--box-size alvik=34`); the predicted box centre is the position, heading is
`None`. Use the same `split.json` for a fair comparison:

```
python baseline_tv.py train --labels D:/recordings --out runs/base --split runs/v1/split.json
```

The COCO weights download from download.pytorch.org on first use. If that is
blocked, copy the file into `~/.cache/torch/hub/checkpoints/`, or use
`--weights imagenet` (backbone only) or `--weights none` (random start; much
weaker unless trained far longer).

### YOLO comparison (research only, not part of the program)

`baseline_yolo.py` does the same as `baseline_tv.py` with Ultralytics YOLO
(default YOLO11n, 320 input): fixed-size boxes from the centre labels, box
centre = position, heading `None`. It is there only so the report can say how
our detector compares with the best-known off-the-shelf detector. Install it
separately (`pip install -r requirements-research.txt`, from the repo root);
nothing else in the repo imports it (a test checks this).

```
python baseline_yolo.py train --labels D:/recordings --out runs/yolo --split runs/v1/split.json --max-minutes 40
python baseline_yolo.py train ... --weights none          # random start, like our model
python baseline_yolo.py train ... --task pose             # also learns the heading (see below)
python evaluate.py ... --model baseline_yolo:runs/yolo/best.pt
python benchmark.py --detector baseline_yolo --weights runs/yolo/best.pt --threads 1,4
```

`--weights coco` (default) starts from COCO-pretrained weights downloaded from
GitHub (`yolo11n.pt`); if blocked, download it elsewhere and pass
`--weights-dir`. `--max-minutes` is a training-time budget (Ultralytics fits
its epochs and learning-rate schedule to it). `--model yolo26n` / `yolov8n`
pick other small models.

**Heading option (`--task pose`)**: YOLO's keypoint model with one keypoint
placed at the same front point our model uses (0.35 x robot size ahead of the
centre, only where `heading_known` is 1). The heading is the direction from
the box centre to that keypoint, `None` if its confidence is under 0.5. It
needs no extra labels and costs almost nothing at run time. The oriented-box
model (`-obb`) was not used: a square box's angle repeats every 90 deg, and
even a long box cannot tell front from back.

### Licences

| Part | Licence |
| --- | --- |
| PyTorch | BSD-3-Clause (BSD-style) |
| torchvision (code and the SSDlite / Faster R-CNN definitions) | BSD-3-Clause |
| ONNX Runtime | MIT |
| ONNX | Apache-2.0 |
| OpenCV (4.5 and later) | Apache-2.0 |
| NumPy | BSD-3-Clause |

All permissive: they allow use, modification and closed-source or commercial
use with attribution. The COCO-pretrained weights were trained on the COCO
dataset (annotations CC BY 4.0, images under their Flickr owners' terms); check
those terms before any commercial use of the baseline. Our own model
(`CentreNet`) uses no pretrained weights.

**Ultralytics YOLO is used for the comparison only and is not part of the
program.** It is licensed AGPL-3.0: anyone who distributes software built on
it, or lets others use a modified version over a network, must release that
whole program's source under AGPL-3.0, unless they buy Ultralytics' paid
enterprise licence. Ultralytics' licensing page says models trained with its code
are covered too. For a factory deployment that is not acceptable, so:

- `ultralytics` is only in `requirements-research.txt`, never in
  `requirements.txt`; only `baseline_yolo.py` imports it, and nothing in the
  program imports `baseline_yolo.py` (`tests/test_baseline_yolo.py` fails if
  that changes)
- running it privately to produce numbers for the report is ordinary use, which
  AGPL-3.0 allows
- **YOLO weights trained here (`runs/yolo*/best.pt`) must not be distributed
  with the program** or put on the robots' Pi; keep them out of releases
- the delivered detector is our own `CentreNet` (no YOLO code or weights)

The COCO-pretrained YOLO weights carry the same AGPL-3.0 terms and the COCO
dataset terms above.

## Adding a class

Classes are a list, one heat map each. Example: adding `forklift`.

1. Label it: rows in `labels.csv` with `cls` = `forklift` (the labelling tool
   needs to offer it too).
2. Train with the class **appended at the end** (never reorder: channel 0 must
   stay alvik), its rough size in camera pixels, and whether it has a front:

   ```
   python train.py --labels D:/recordings --out runs/v2 \
       --classes alvik,hand,unknown,machine,forklift --obj-size forklift=120 \
       --heading-classes alvik,machine,forklift \
       --init-from runs/v1/best.pt --split runs/v1/split.json
   ```

   `--init-from` copies everything the old model learned (the new class's map
   starts fresh), so it needs less data and time than starting over.
3. Export as usual; `infer.py` reads the class list from the `.onnx` file, so
   nothing else changes.

Classes with no labels yet (e.g. `machine` today) simply learn "never here";
they cost almost nothing at run time.

## Tests

```
python -m pytest tools/detector/tests -q
```

They check: headings turn and mirror correctly with the picture (including a
picture-level check after random augmentation), label filtering rules, split by
recording, decoding and scoring maths, the homography inversion, export
(PyTorch and ONNX Runtime agree), adding a class with `--init-from`, a short
train -> export -> evaluate -> benchmark run on synthetic data, the
baselines' interfaces, and that `ultralytics` stays out of the program (the
YOLO training tests are skipped when `ultralytics` is not installed).

## Results so far (synthetic data only)

No real labelled frames existed yet, so the pipeline was tested on 12
synthetic recordings x 150 frames (8 train / 2 val / 2 test recordings),
trained on a 2-core shared CPU for 40 minutes (600 steps, stopped by the time
limit while still improving). Test recordings, threshold 0.3, radius 8 px:

| | our model (ONNX) | our model (INT8 ONNX) | SSDlite baseline (random start) |
| --- | --- | --- | --- |
| alvik precision / recall / F1 | 0.986 / 0.998 / 0.992 | 0.973 / 1.000 / 0.986 | 0.921 / 0.997 / 0.957 |
| alvik centre error, mean / p95 | 1.5 / 4.4 px | 1.6 / 4.5 px | 2.3 / 4.1 px |
| alvik heading error, median / p95 | 8.6 / 26 deg (0.6% flipped) | 8.8 / 26 deg | none (no heading) |
| unknown (cups) F1 | 0.74 | 0.66 | 0.23 |
| time per frame, 320x240, 1 thread | 4.0 ms | 3.7 ms | 37 ms (PyTorch) |

Hands: the test recordings happened to contain none; on the validation
recordings hand F1 was 0.13 at 8 px and 0.50 at 20 px (only ~100 training
examples; a hand's centre is vague). These numbers only show the code works
and learns; they say nothing about the real camera.

## Synthetic data

`python make_synthetic.py --out /tmp/synth --recordings 12 --frames 150` draws
fake overhead frames in the labels format (floor with tape grid, dark
rounded-square robots with two front LEDs, hands, cups, empty frames, a few
pending and rejected rows). It exists only to test the code; scores on it say
nothing about the real camera.
