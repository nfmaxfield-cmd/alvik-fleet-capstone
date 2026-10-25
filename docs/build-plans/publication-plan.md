# Publication plan: what MIRLT could publish, where, and what it still needs (2026-10-25)

**Premise.** Nick, 2026-10-24: "I want to be published." This plan places MIRLT against the published work, says
plainly what is new and what is not, picks the one or two claims the data can carry, lists the experiments and
statistics a reviewer would ask for, and gives venues, an 8-week timeline and a section-by-section outline.
Every source in the body was opened. The links are in Appendix A.

---

## 1. Where MIRLT stands

| Area | Closest published work | What that work needs | What MIRLT does differently | New? |
|---|---|---|---|---|
| Odometry self-calibration | UMBmark (Borenstein & Feng 1996): a 4 m square both ways, 10-22x lower systematic error. Roy & Thrun (ICRA 1999): online, from the robot's own sensors, about 10x. Censi et al. (T-RO 2013): wheel radii, baseline and sensor pose, closed form, no external device | A measured path, or an on-board exteroceptive sensor (laser) | Calibrates from an **uncalibrated external camera** that is calibrated at the same time | **No** for "calibration cuts odometry error about 10x": it is expected. Session 1's 21.0 to 1.4 cm is the same size as UMBmark's gain |
| Robot as the camera's calibration object | Rekleitis & Dudek (2005): the robot carries a 6-face checkerboard target; inter-camera distances 2.1 cm mean error. Meger, Rekleitis & Dudek (SPLAM): ARTag panels on the robot. Chakravarty & Jarvis (ACRA 2009): **no target**, but the robot is localised by laser on a known map; 38-52 cm mean error | A target on the robot, or a robot that already knows where it is | **Neither side is calibrated at the start.** One joint fit gives the camera (focal length, lens term, pose), every robot's wheel and gyro scale, and the arena's edges. Scale comes from the wheels' average and the robots' known size. Edges come from touch | **Possibly.** This search found no paper doing exactly this. That does not prove none exists (section 2) |
| Camera / multi-sensor extrinsics | Kalibr thesis (Rehder, ETH 2018): camera-IMU-laser, needs a checkerboard. MEMROC (Allegro et al.): cameras on the robot, needs a pattern. Censi 2013 and Kümmerle et al. (IROS 2011, as cited by MEMROC and Nutalapati et al.): motion-based, sensors on the robot | Sensors carried by the robot, or a pattern | The camera is fixed in the room; the robots move | Different problem; cite as background |
| Adaptive noise and sensor fault detection | Mehra (IEEE TAC 1970): noise variances identified from the filter's innovations. Mohamed (Calgary 1999): innovation-based adaptive estimation in a moving window. Särkkä & Nummenmaa: variational Bayes noise adaptation. Roumeliotis, Sukhatme & Bekey (MMAE bank of filters) and Goel et al. (ICRA 2000): fault identification on mobile robots. Duenas Arana et al. (ICRA 2019): chi-squared on innovations, integrity risk | A filter whose innovations show the noise; faults modelled in advance (MMAE) | Each sensor's noise is learned against other sensors. The **fleet** separates the camera's share (common to all robots) from each robot's own (`e02_trust`). A degraded sensor is flagged with no fault model given in advance | **Incremental at most.** The shared-versus-own split across a fleet may be a small new point. Adaptive noise and innovation tests are 50 years old |
| Low-cost multi-robot testbeds | Robotarium (Pickem et al.): webcam + ArUco, GRITSBots about $60-100, up to 20 robots, sim-to-hardware error 5 mm. GRITSBot testbed: AprilTags, automatic charging. Duckietown (Paull et al.): $150 robots, camera only, planar homography. Kilobot: $14 in parts, no odometry. Zooids (UIST 2016): projector gray-code, 73 Hz. HeRo 2.0: $18, AprilTag camera, OptiTrack truth | **Markers** (ArUco, AprilTag), a projector, or motion capture | No markers and no lab-grade truth. Localisation comes from fusing on-board sensors with a markerless tracker, and the system calibrates itself | **The combination is the selling point**, not any one part |
| Educational robots | e-puck (Mondada et al. 2009), Thymio (IEEE RAM 2017), Duckietown, the Arduino Alvik (ToF 8x8, 3 line sensors, LSM6DSOX IMU, 3 ppr encoders, Nano ESP32) | Classroom use, typically with markers or tape for localisation | An off-the-shelf classroom robot reaches about 1-3 cm return accuracy with no markers and no hand calibration | An **application angle** for an education or automation venue |

## 2. Is it novel? Plainly

- **Not new:** self-calibrating odometry; the size of the gain; adaptive Kalman filtering; innovation-based fault
  detection; cheap swarm testbeds; using a robot to calibrate a fixed camera.
- **Possibly new, and the paper's core:** joint, target-free calibration where *nothing* starts calibrated (the
  camera's intrinsics and pose, every robot's wheel and gyro scale, the arena's boundaries), done online by a
  fleet of commodity robots with no markers. Rekleitis & Dudek needed a checkerboard on the robot. Chakravarty &
  Jarvis needed a laser-localised robot on a prior map, and reached half-metre accuracy. MIRLT's explore agreed
  with the corner clicks to 2.9 cm median on real data (2026-10-23), within 1.1 cm of the true camera in the
  simulator.
- **Before any submission:** a systematic search. Forward citations of Rekleitis & Dudek 2005, Chakravarty &
  Jarvis 2009 and Censi 2013 in Google Scholar; the terms "camera network calibration mobile robot", "targetless",
  "self-calibration of environment cameras". One likely-relevant paper (Robotics and Autonomous Systems, "Self-
  calibration of environmental camera for mobile robot navigation") could not be opened (ScienceDirect blocked the
  fetch). It must be read before claiming novelty.

## 3. The claims a paper could make

**Claim 1 (main).** A fleet of uncalibrated, markerless, low-cost robots and an uncalibrated overhead webcam
calibrate each other from the robots' own driving, with no target, map or measuring tape. The result is
equivalent to a conventional target-based calibration, both in the camera's floor error and in the robots'
navigation error.

| Evidence | Have | Still needed |
|---|---|---|
| Robots' navigation error falls with self-calibration | Session 1 (2026-10-23, 45 runs): end error U 21.0 cm (n 17) vs C 1.37 (n 8), Holm-adjusted permutation p < 0.001, d 1.15. Pi stages 1-2 (2026-10-24, 30 runs): C 3.4 vs U 21.9 cm | Replication across sessions (power cycles). Truth that is not the same camera |
| The camera calibrated by the robots alone | Explore vs corner clicks: 2.9 cm median, scale 6% apart (real, 2026-10-23); 1.1 cm median vs the true camera (simulator) | Tape-mark truth. A conventional baseline (checkerboard + clicks). Several camera poses |
| Equivalence to conventional | None | The baseline arm (section 4, Exp 1-2) |

**Claim 2 (secondary, or a second paper).** The system learns how much to trust each sensor, and splits the
camera's error from each robot's using the fleet. A deliberately degraded sensor is flagged within a bounded time,
false alarms on healthy sessions stay rare, and localisation accuracy under the fault stays close to the healthy
level.

| Evidence | Have | Still needed |
|---|---|---|
| Trust code built, with a degrade harness | `e02_trust.py`, `e02_degrade.py`; replay of stages 1-2 | Real degraded sessions; detection rate and delay; false alarms per hour |
| False-alarm behaviour | The replay flagged healthy robots ("the flags are mostly the method": the median robot sets the camera's share). With the 3 cm jump cut: none on the laptop morning; two on stage 2 that cleared within 2 minutes | A fixed rule, then a count over at least 10 healthy hours |

**Recommendation:** lead with claim 1; put claim 2 in as a short "robustness" section only if its data are in by
week 5. Otherwise keep claim 2 for a second paper (CASE or a workshop).

## 4. Experiments and statistics a reviewer would ask for

| # | Experiment | Design | Sample size (and why) | Statistics |
|---|---|---|---|---|
| 1 | **Camera calibration accuracy**: explore (robots only) vs conventional (printed checkerboard + 4 corner clicks, `classic_calibration`) | Each camera pose calibrated both ways; a robot placed on each of the 7 tape marks (`e02_camerr.MARKS`) | At least 3 camera poses x 7 marks = 21 paired points per method. With the sd of differences about 1 cm, a paired equivalence test at a 1 cm margin needs 9 pairs (Appendix B) | Paired TOST (two one-sided tests) on per-mark error, margin 1 cm; RMS and worst error per pose; Bland-Altman plot |
| 2 | **Navigation accuracy**: U, C (self-calibrated), C-conv (corrections from the conventionally calibrated camera), KC; K only as a reference | Shuffled blocks as now; balanced over robot x route; **at least 3 sessions on different days** (power cycles: the robot-level replicate) | C vs C-conv equivalence at 1 cm: per-run sd 0.8-1.5 cm (session 1: C 0.79, KC 1.54) gives 18-39 runs per arm if independent, 9-20 pairs if paired by robot x route x block. U vs C: already overwhelming; 8 per robot is enough | Linear mixed model, end error ~ condition + route, with random intercepts for robot and session (the "mixed model" already listed as a needed test). TOST for C vs C-conv. Report robot-level means: 3 robots is n = 3 for "any robot" |
| 3 | **Camera-free truth for every run** | Return-to-start by picture-to-picture comparison (no calibration needed), and the find's board-touch pose (1-2 cm per the session analysis; to be measured) | Same runs as 2 | Agreement of the camera's end error with these, per condition |
| 4 | **Degraded sensor** (claim 2) | Simulator grid first (camera noise 1/2/3 cm, wheel jitter 2/5%, gyro jitter 1/3%, front bias 5 cm), then real: 2 levels per sensor | At least 20 injected trials per sensor and level that will be claimed: 20 of 20 detected gives a 95% lower bound of 0.84 (Wilson). Healthy time at least 10 h: 0 false alarms then bounds the rate at 0.3 an hour (rule of three) | Detection rate with Wilson interval; time to flag (median, IQR); accuracy with trust on vs off under the fault (paired, at least 10 runs per arm) |
| 5 | **Generality** | Camera pose (as now / moved and tilted) x floor (plywood / second surface), explore from scratch each time (`generality-audit.md`, demonstration) | 4 cells x 7 marks; 8 C runs per cell | Per-cell mark error and C end error; a two-factor ANOVA, or the mixed model with cell terms |
| 6 | **Ablations** | Explore without the robot-size term; without touch edges; with 1 vs 3 robots | 3 sessions in the simulator plus 1 real | Mark error and scale error per ablation |

<sub>TOST (two one-sided tests): shows two methods agree within a stated margin, instead of only failing to find a
difference. Bland-Altman plot: each pair's difference against its mean, with the mean difference and its 95% limits.
Linear mixed model: a regression that also lets each robot and each session have its own baseline, so repeated runs
of one robot are not counted as independent evidence. Wilson interval: a confidence range for a success rate that
behaves well near 100%. Rule of three: with zero events in T hours, the 95% upper bound on the rate is 3 / T.</sub>

## 5. Threats to validity

| Kind | Threat | What it does | Remedy |
|---|---|---|---|
| Internal | **The camera is both teacher and judge**: C learns from the camera, and the analysis scores C with the same camera | Inflates C's advantage when the camera's error is shared | Exp 1 and 3 (tape marks, board touches, pixel return) |
| Internal | Block 0 is all U (randomisation restricted) | Order, battery and warm-up confound U vs C | Report run order; error vs run number within robot (already a listed test); one session with U spread through |
| Internal | The trust code's healthy values were set from the same sessions it is tested on | k about 1 by construction | Fix them before the claim-2 sessions; test on new sessions only |
| Construct | **K's heading error (mean 83 deg) is probably a measurement artefact.** The heading is known only up to a quarter turn and resolved from the robot's own record, which K ignores | Makes K look far worse than it is | Resolve the quarter from the camera track in K; drop K's heading from the claims until fixed |
| Construct | End error is a return-to-start error | Cancels most absolute camera error, and favours a gyro-scale correction | Also report far-point error against tape truth |
| Construct | Absolute scale: explore and clicks differ by 6% (2026-10-23) | A 6% scale error is 6 cm over 1 m | Exp 1 decides which is right; a single tape distance if needed |
| External | One room, one camera, one robot model, routes of at most 100 cm, 3 robots | Generality is asserted, not shown | Exp 5; say plainly what was not tested |
| Statistical | Cells of 2-3 runs per route x condition; Holm correction within each table only; Welch and Mann-Whitney treat runs as independent | Overstated certainty | The mixed model as primary, the permutation test as a check; one primary outcome named before the sessions |
| Statistical | The 1.5 x IQR outlier rule is chosen | Different outliers under another rule | Report with and without (already done) |
| Novelty | The literature already shows calibration cuts odometry error about 10x (UMBmark 1996, Roy & Thrun 1999) | U vs C alone is an expected result | Frame the paper around *target-free joint* calibration and equivalence to conventional, not around U vs C |

## 6. Venues

| Venue | Type | Deadline | Event | Fit |
|---|---|---|---|---|
| **IISE Annual Conference 2027** | 250-word abstract (not indexed) or a reviewed research paper (indexed via ProQuest) | Abstracts **8 November** (year not stated on the page found; iise.org could not be opened: confirm today). In 2026 the cycle was abstracts 10 Nov 2025, papers 18 Jan 2026 | 22-25 May 2027, Louisville KY | ISE home venue; "Manufacturing Automation and Robotics" topic. **Cheapest first step** |
| **NCUR 2027** | Undergraduate abstract and presentation | **4 December 2026** | 12-14 April 2027, Grand Rapids MI | Undergraduate venue; recent graduates may present work done as an undergraduate |
| **arXiv (cs.RO)** | Preprint | Any time | n/a | First public record. **Needs an endorser** (since 21 Jan 2026 an institutional email is not enough) |
| **IEEE CASE 2027** | Full paper (peer reviewed, IEEE Xplore) | **1 March 2027** (work-in-progress and industry papers 1 April 2027) | 23-27 August 2027, Linz, Austria | Best main-track fit: automation science, ISE-friendly, testbeds welcome |
| IROS 2027 | Full paper | 1 March 2027 | 26 Sept-1 Oct 2027, Florence | Harder; needs the conventional baseline and a second setting to have a chance |
| ICRA 2027 workshops | Short workshop papers | Not posted. Main papers closed 15 Sept 2026; workshops are announced 31 Jan 2027 | 24-28 May 2027 (Seoul, per the site's pages) | Good for claim 2 or the testbed; watch for multi-robot, calibration or education workshops |
| RSS 2027 workshops | Short papers | Not released (the 2026 main deadline was late January) | 6-11 July 2027, Athens | Same as ICRA workshops |
| IEEE MIT URTC 2027 | Undergraduate paper (IEEE) | About August 2027 (2026: 9 August) | 8-10 October 2027, Cambridge MA | Undergraduate-only. Eligibility after graduation is not stated |
| IEEE RA-L | Journal letter, 6 pages (+2 paid) | Year-round | Optional conference presentation | Later, if claim 1 plus Exp 5 come out strongly |

**Recommended path:** IISE abstract (if 8 Nov 2026 is confirmed) and NCUR abstract (4 Dec) as the student track.
An arXiv preprint in mid-December. A CASE 2027 full paper by 1 March 2027 (the preprint grows into it). An ICRA or
RSS workshop paper for claim 2 when calls appear.

## 7. Authorship and advisor

- **Everyone named must agree before anything is submitted, including arXiv.** Who counts here is Nick's call
  with Professor Sodhi. Possible co-authors: any registered capstone group members; Professor Sodhi (advisor,
  orders the parts); Andrew Muszynski (the predecessor system and shared data; his repos carry no licence, so any
  of his code or data used needs his written permission).
- Agree the author order, the corresponding author and the affiliation in one written message before week 3.
- Ask Professor Sodhi whether the university has rules for publishing course work, and whether he will endorse on
  arXiv (or knows an endorser in cs.RO).
- IEEE requires AI-generated text, figures and code to be **disclosed in the acknowledgments**, naming the system
  and the sections. This repository's code and these plans were written with Claude. That has to be stated.
- The repo is public with no licence (`generality-audit.md` C4). Pick one before the preprint, so the artifact can
  be reused.

## 8. Timeline (8 weeks, 26 October to 20 December 2026)

| Week | Dates | Work | Output |
|---|---|---|---|
| 1 | 26 Oct-1 Nov | Authorship talk with Professor Sodhi; confirm the IISE deadline; tape the 7 marks; pre-register the primary outcome and the analysis (one page in `docs/decisions/`); systematic literature search (section 2) | Author list; analysis plan; search notes |
| 2 | 2-8 Nov | Build audit ranks 1 and 4 (robot profile, `site.json`, independent truth, balanced schedule); fix the K heading quarter; IISE abstract | IISE abstract submitted (if confirmed) |
| 3 | 9-15 Nov | Sessions: Exp 1 (3 camera poses, both methods) and the first Exp 2 session with C-conv | Camera accuracy data |
| 4 | 16-22 Nov | Exp 2 sessions 2-3; surface check build (R7); simulator grid for Exp 4 | Navigation data complete |
| 5 | 23-29 Nov | Exp 5 (2 x 2 generality); real degraded runs if claim 2 stays in; analysis scripts final | All data in; decide claim 2 in or out |
| 6 | 30 Nov-6 Dec | Analysis, figures; full draft; NCUR abstract (4 Dec) | Draft v1; NCUR submitted |
| 7 | 7-13 Dec | Advisor and co-author review; an internal "reviewer" pass against section 5; fixes | Draft v2 |
| 8 | 14-20 Dec | Final presentation material reuses the figures; arXiv endorsement and posting once everyone has approved | arXiv preprint |
| After | Jan-1 Mar 2027 | Grow into the CASE paper (and IROS if Exp 5 is strong) | CASE submission |

## 9. Paper outline (CASE format; check the 2027 page limit when the call is posted)

| Section | Content | Figure / table | Data source |
|---|---|---|---|
| Title, abstract | "Robots as their own calibration target: joint target-free calibration of an overhead camera and a low-cost robot fleet" | none | |
| I. Introduction | The problem (markers, targets and hand calibration cost labour); what is known at the start (flat floor, the robots' own size and sensor positions, a camera somewhere above); contributions (claim 1, claim 2 if in, open code) | Fig 1: the board, the robots and the camera's view, with the unknowns marked | Existing footage (2026-10-23) |
| II. Related work | Section 1 of this plan, condensed | Table I: method vs needs a target / a reference sensor / calibrates the camera / calibrates the robots / detects faults / cost | This plan |
| III. System | Robots, camera, Pi, firmware protocol, markerless tracker (detector + lock-on), recovery ladder, virtual border | Fig 2: architecture; Table II: hardware and cost | Repo docs |
| IV. Method | A. Explore joint fit (pinhole + k1, per-robot s and g, sum of log s = 0, size check, touch edges). B. Per-run refit from straight stretches (with the 3 cm jump cut). C. Trust: shared vs own terms; the flag rule | Fig 3: one explore session (rays, stops, fitted edges) | Session 2026-10-24 stage 3, or a new explore |
| V-A. Exp 1: camera accuracy | Explore vs conventional, at the tape marks | Fig 4: error at each mark, both methods, per camera pose; Table III: RMS, worst, TOST | **Planned** (week 3) |
| V-B. Exp 2: navigation accuracy | U / C / C-conv / KC, mixed model | Fig 5: every run's end error by condition (dots + median: the existing chart style); Table IV: model estimates | 2026-10-23 S1 and 2026-10-24 Pi stages (pilot); **planned** sessions (weeks 3-4) |
| V-C. Exp 5: generality | 2 x 2 camera pose x floor | Fig 6: mark error and C end error per cell | **Planned** (week 5) |
| V-D. Exp 4: robustness (if claim 2) | Injected faults | Fig 7: trust factor k over time with the fault's start marked; Table V: detection rate, delay, false alarms an hour | Simulator grid + **planned** real runs |
| VI. Discussion and limits | Scale, flat floor, one robot model, robots moving one at a time, the simulator's optimism | none | Section 5 of this plan |
| VII. Conclusion | | none | |
| Acknowledgments | Advisor, Andrew's predecessor system and data, **AI-use disclosure** | none | |

---

## Appendix A. Sources (all opened for this plan)

Odometry and joint calibration
- [Measurement and Correction of Systematic Odometry Errors in Mobile Robots (Borenstein & Feng, IEEE T-RA 1996)](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- [Online Self-Calibration for Mobile Robots (Roy & Thrun, ICRA 1999)](https://www.ri.cmu.edu/publications/online-self-calibration-for-mobile-robots)
- [Simultaneous Calibration of Odometry and Sensor Parameters for Mobile Robots (Censi et al., IEEE T-RO 2013)](https://iris.uniroma1.it/handle/11573/495656)
- [Model Free Calibration of Wheeled Robots Using Gaussian Process (Nutalapati et al.; cites Kümmerle et al. IROS 2011)](https://arxiv.org/pdf/1910.11917)
- [A Generalized Framework for Autonomous Calibration of Wheeled Mobile Robots (Nutalapati et al.)](https://arxiv.org/pdf/2001.01555)

Cameras calibrated with robots, and extrinsics
- [Automated Calibration of a Camera Sensor Network (Rekleitis & Dudek)](https://facerec.ece.wisc.edu/cameranet/rekleitis05.pdf)
- [Simultaneous Planning, Localization, and Mapping in a Camera Sensor Network (Meger, Rekleitis & Dudek)](https://www.cs.ubc.ca/labs/lci/curious_george/pubs/DARS_2006.pdf)
- [External Cameras & A Mobile Robot: A Collaborative Surveillance System (Chakravarty & Jarvis, ACRA 2009)](https://www.araa.asn.au/acra/acra2009/papers/pap135s1.pdf)
- [MEMROC: Multi-Eye to Mobile RObot Calibration (Allegro, Terreran & Ghidoni)](https://arxiv.org/html/2410.08805)
- [Multi-Sensor System Calibrations (Rehder, ETH thesis 2018; Kalibr)](https://www.research-collection.ethz.ch/server/api/core/bitstreams/f3263ec4-4c7d-4d62-8836-4e7d23ac9d67/content)
- [A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration (IEEE T-RA 1989)](https://research.ibm.com/publications/a-new-technique-for-fully-autonomous-and-efficient-3d-robotics-handeye-calibration)
- [Mobile Robot Camera Extrinsic Parameters Auto Calibration by Spiral Motion (Ekmanis & Ņikitenko 2016)](https://ortus.rtu.lv/science/lv/publications/22248)
- [Learning Surveillance Tracking Models for the Self-Calibrated Ground Plane (Renno et al., BMVC 2002)](https://bmva-archive.org.uk/bmvc/2002/papers/185/full_185.pdf)
- [Automated Ground-Plane Estimation for Trajectory Rectification (Hales et al., CAIP 2013)](https://research.aber.ac.uk/en/publications/automated-ground-plane-estimation-for-trajectory-rectification/)

Adaptive filtering, fault detection, trust
- [On the Identification of Variances and Adaptive Kalman Filtering (Mehra, IEEE TAC 1970)](https://swov.nl/en/publicatie/identification-variances-and-adaptive-kalman-filtering)
- [Optimizing the Estimation Procedure in INS/GPS Integration (Mohamed, Calgary 1999: innovation-based adaptive estimation)](https://ucalgary.ca/engo_webdocs/KPS/99.20127.AMohamed.pdf)
- [Recursive Noise Adaptive Kalman Filtering by Variational Bayesian Approximations (Särkkä & Nummenmaa)](https://users.aalto.fi/%7Essarkka/pub/vb-akf-ieee.pdf)
- [Sensor Fault Detection and Identification in a Mobile Robot (Roumeliotis, Sukhatme & Bekey)](https://www-users.cse.umn.edu/~stergios/papers/abstracts/abstract_iros98.html)
- [Fault Detection and Identification in a Mobile Robot Using Multiple Model Estimation and Neural Network (Goel et al.)](https://www-users.cse.umn.edu/~stergios/papers/icra00_fault.pdf)
- [Recursive Integrity Monitoring for Mobile Robot Localization Safety (Duenas Arana et al., ICRA 2019)](https://www.aoe.vt.edu/content/dam/aoe_vt_edu/people/faculty/joerger/publications/Recursive_Integrity_Monitoring_for_Mobile_Robot_Localization_Safety_2019.pdf)
- [Distributed Multi-Robot Localization (Roumeliotis & Bekey)](https://www-users.cse.umn.edu/~stergios/papers/abstracts/abstract_TRA01.html)

Testbeds and educational robots
- [The Robotarium: A Remotely Accessible Swarm Robotics Research Testbed (Pickem et al.)](https://arxiv.org/pdf/1609.04730)
- [The GRITSBot in its Natural Habitat: A Multi-Robot Testbed (Pickem, Lee & Egerstedt)](https://gritslab.gatech.edu/home/wp-content/uploads/2016/03/2015-The-GRITSBot-in-its-natural-habitat-A-multi-robot-testbed.pdf)
- [Duckietown: an Open, Inexpensive and Flexible Platform for Autonomy Education and Research (Paull et al.)](https://people.csail.mit.edu/hangzhao/papers/duckietown.pdf)
- [Kilobot: A Low Cost Scalable Robot System for Collective Behaviors (Rubenstein, Ahler & Nagpal)](https://users.cs.northwestern.edu/~mrubenst/ICRA12.pdf)
- [Zooids: Building Blocks for Swarm User Interfaces (Le Goc et al., UIST 2016)](https://aviz.fr/wiki/uploads/Research/swarmui.pdf)
- [HeRo 2.0: A Low-Cost Robot for Swarm Robotics Research (Rezeck et al.)](https://arxiv.org/pdf/2202.12391)
- [The e-puck, a Robot Designed for Education in Engineering (Mondada et al. 2009)](https://infoscience.epfl.ch/record/135236)
- [Bringing Robotics into Formal Education Using the Thymio Open Source Hardware Robot (Mondada et al., IEEE RAM 2017)](https://infoscience.epfl.ch/record/223049)
- [Arduino Alvik datasheet (AKX00066)](https://docs.arduino.cc/resources/datasheets/AKX00066-datasheet.pdf)

Venues and policies
- [IEEE CASE 2027 (event listing)](https://www.beri.net/events/ieee-case-2027) and [IEEE RAS important dates](https://www.ieee-ras.org/events/category/important-dates/)
- [IROS 2027 (event listing)](https://www.beri.net/events/iros-2027)
- [ICRA 2027: contribute / dates](https://2027.ieee-icra.org/contribute/) and [call for technical papers](https://2027.ieee-icra.org/announcements/call-for-technical-papers/)
- [RSS 2027 (event listing)](https://www.beri.net/events/robotics-science-and-systems-2027)
- [IISE Conference & Expo 2027 (event listing)](https://cantonfair.net/event/24434-iise-conference-expo) and [IISE 2026 call for abstracts, Manufacturing & Design](https://www.iise.org/uploadedFiles/Annual_Conference/Program/IISE%20Annual%20Conference%202026%20-%20Call%20for%20Abstracts%20Manufacturing%20and%20Design.pdf)
- [NCUR 2027 FAQs](https://ncur.secure-platform.com/2027/page/faqs)
- [IEEE MIT URTC](https://urtc.mit.edu/)
- [RA-L information for authors](https://www.ieee-ras.org/publications/ra-l/ra-l-information-for-authors/)
- [arXiv 2026 endorsement policy changes (CASRAI summary)](https://casrai.org/news/arxiv-2026-endorsement-english-ai-policy-changes)
- [IEEE author guidelines for AI-generated text](https://open.ieee.org/author-guidelines-for-artificial-intelligence-ai-generated-text/)

Could not be opened (blocked during this research; worth reading by hand):
`https://www.sciencedirect.com/science/article/abs/pii/S092188900600159X` (Self-calibration of environmental camera
for mobile robot navigation), `https://www.iise.org/annual/` (the official 2027 IISE dates),
`https://authors.library.caltech.edu/records/b4rqz-r2t27` (Censi 2013 record; the Sapienza record was used instead).

## Appendix B. Sample-size arithmetic
- Equivalence (TOST), two independent arms, alpha 0.05 each side, power 0.8, true difference 0:
  n per arm = 2 sd² (z0.95 + z0.90)² / margin². With margin 1 cm: sd 1.0 gives 18; sd 1.5 gives 39; sd 2.0 gives 69.
  With margin 0.5 cm: 69 / 155 / 275 (too many: keep the margin at 1 cm).
- Paired TOST, margin 1 cm: sd of differences 1.0 gives 9 pairs; 1.5 gives 20.
- Wilson 95% interval: 20 of 20 detected gives 0.84-1.00; 19 of 20 gives 0.76-0.99; 30 of 30 gives 0.89-1.00.
- Zero false alarms in T hours: 95% upper bound 3 / T (rule of three): 0.6 an hour at 5 h, 0.3 at 10 h, 0.15 at 20 h.
- Superiority, two arms, alpha 0.05 two-sided, power 0.8, a 1 cm difference: sd 1.5 gives 36 per arm; sd 2.0 gives 63.
<sub>TOST: two one-sided tests; it shows two methods are equivalent within a stated margin, rather than failing to
show a difference.</sub>

---

## New tests that may now be needed
1. **Conventional baseline arm (C-conv):** the camera calibrated by checkerboard and clicks, the robots
   calibrated from it, then the same routes.
2. **Tape-mark camera check** at 7 marks for each camera pose (both calibration methods).
3. **K heading fix:** resolve the quarter turn from the camera track in K runs, then re-score session 1's K
   headings.
4. **Order effects:** end error against run number within each robot, and one session with U spread through the
   blocks.
5. **Healthy false-alarm count** for the trust flags over at least 10 h, with the healthy values frozen first.
6. **Explore ablations:** no size term, no touch edges, 1 robot vs 3 (simulator, then one real session).
7. **Absolute scale check:** one tape distance across the board against explore's and the clicks' scales (they
   differ by 6%).

## Possible inaccuracies
- **Novelty is judged from one search session** of about 40 queries and 35 pages. One likely-relevant paper could
  not be opened. "Possibly new" should not go in any abstract until the systematic search (section 2) is done.
- **Venue dates** for CASE, IROS, RSS and IISE come from aggregator listings (beri.net, cantonfair.net) and the
  RAS dates page, not from each conference's own call. The IISE "8 November" has no year on the page found.
  ICRA 2027's location is inferred from its site's pages ("Visit Seoul").
- **Session numbers differ by source.** Session 1's report gives U 21.0 cm (n 17) and C 1.37 (n 8). The summary
  outline gives 16.4 vs 1.4 (44 runs), which matches the report's U mean without its outlier (16.3). Pick one
  definition before writing.
- **The sd values used for sample sizes** (0.8-1.5 cm) come from one session's C and KC runs of 7-8 each; the
  sizes move with the square of sd.
- **The 1 cm equivalence margin is chosen**, not derived. A design of experiments does not set it. It should come
  from the use: what accuracy a task needs, for example the dock's funnel width or the tight-gap demo's clearance.
- **"About 1 mm" for the board-touch pose** is the trust code's own description (`e02_trust.py` docstring); the
  analysis puts a touch at about 1-2 cm from tyre slip. Treat the touch as 1-2 cm until measured.
- **Explore's 2.9 cm median agreement** is against the corner clicks, which are not truth either.
- **Authorship facts** (who is in the capstone group) are not in the repository; section 7 lists roles only.
- **Chosen minimums:** 3 camera poses, 3 sessions, 20 injected trials and 10 healthy hours are the smallest numbers
  that the Appendix B arithmetic supports, not values derived from a target. A design of experiments is not the
  right tool for them; a power calculation from the first new session's sd is, and it should replace them in week 3.
- **The timeline assumes** about 3 sessions a week with Nick home, and that the rank 1 and 4 builds take the
  estimated 12-15 h. Both are guesses.
