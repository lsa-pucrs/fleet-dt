# Frame-size traces of the video topic

The video topic of the experiments (paper Section VI-C, Sources) is an output topic whose outputs are
JPEG frames of a side-by-side stereo image, encoded as `vessel/publishers/zed_publisher.py` encodes the
frames of the ZED. `fdt_source.py --size-trace FILE` replays one of these files (one size in bytes per
line, `#` comments) cyclically, from a start position that the seed of the run fixes, one size per output.

The experiments (E0 to E4, `emu/experiments/matrix.py`, TRACES = "water", scale 0.5) use the measured
sequence of a stereo camera over water, `water-sbs-q<Q>-s0p5.txt`; the traces of the camera of Pintado
(`zed-*`, `still-*`, below) remain as a cross-check of the level and fed the saturation runs (SAT_TRACE).

| File | Content |
|---|---|
| `water-sbs-q<Q>-s<S>.txt` | traces of the experiments: 4500 frames of a recorded stereo sequence over water (Pohang Canal Dataset, below), JPEG quality Q in {50, 70, 90}, scale S in {1 (1344x376), 0.5 (672x188)} |
| `water-members.txt` | the 9000 input members (left and right PNG) with their CRC-32 and time stamps |
| `make_trace_water.py` | the program that produced the water files (no seed: deterministic) |
| `zed-sbs-q<Q>-s<S>.txt` | lognormal AR(1) traces at the median of the frames of the camera of Pintado (2250 frames, model below; not used by the experiments) |
| `still-q<Q>-s<S>.txt` | sizes of the frames of the camera of Pintado under synthetic motion (cross-check) |
| `stats.json` | mean, median, quantiles, coefficient of variation, lognormal fit and lag-1 autocorrelation of the log size of every file ("water", "still", "model") |
| `make_trace.py` | the program that produced the zed and still files (seed 1) |
| `stripe-shares.json` | share profile of the 16 horizontal stripes of the spatial video layers (`vessel/publishers/video_layers.py`) per quality and scale: 24 pairs of the water sequence (012000 to 016499, evenly spread), each side-by-side frame encoded whole and as 16 stripe JPEGs; share w_s = mean stripe bytes / whole-frame bytes, with its CV and maximum over the pairs and the per-pair sizes |
| `make_stripe_profile.py` | the program that produced `stripe-shares.json` (HTTP range requests on the ZIP of the dataset; about 10 s) |

## Water sequence (the traces of the experiments)

| Item | Value |
|---|---|
| Dataset | Pohang Canal Dataset: D. Chung, J. Kim, C. Lee, J. Kim, "Pohang Canal Dataset: A Multimodal Maritime Dataset for Autonomous Navigation in Restricted Waters", Int. J. Robotics Research, 2023, doi:10.1177/02783649231191145 |
| License | CC BY-NC 4.0 (https://creativecommons.org/licenses/by-nc/4.0/), stated on the official page https://sites.google.com/view/pohang-canal-dataset/home and in the AWS Open Data Registry entry (https://registry.opendata.aws/pohang-canal-dataset/, datasets/pohang-canal-dataset.yaml of awslabs/open-data-registry), checked 2026-10-15; research use with attribution, the derived sizes are redistributed under the same terms |
| Source | public bucket s3://pohang-canal-dataset (us-west-2), https://pohang-canal-dataset.s3.us-west-2.amazonaws.com/pohang00/stereo.zip (111,791,870,918 bytes): stereo camera, 2048x1080 per eye, 8-bit PNG, 10 frames/s, 22,184 pairs |
| Sequence | pohang00, pairs 012000 to 016499 (450 s, outer port and near-coastal waters: open water with the shore and vessels; contact sheet of the whole sequence checked) |
| Encoding | each eye: centre crop to the aspect of the ZED WVGA eye (672:376), resize (INTER_AREA) to 672 S x 376 S; side by side [left, right]; `cv2.imencode(".jpg", img, [IMWRITE_JPEG_QUALITY, Q])` (OpenCV 4.13.0, libjpeg-turbo 3.1.2, baseline 4:2:0), as `zed_publisher.py` |
| Reproduction | `python3 emu/traces/make_trace_water.py --out DIR` (HTTP range requests on the ZIP, CRC-32 of every member checked; 12 min) |

| File (S 0.5) | Median | Mean | p05 | p95 | Max | CV | Lag-1 autocorrelation (log) | Pintado median (cross-check) |
|---|---|---|---|---|---|---|---|---|
| Q50 | 10721 | 10735 | 9739 | 12000 | 12540 | 0.060 | 0.995 | 15255 |
| Q70 | 14437 | 14447 | 12914 | 16104 | 16769 | 0.059 | 0.996 | 20012 |
| Q90 | 27048 | 26984 | 23954 | 29755 | 30735 | 0.056 | 0.997 | 35532 |

At scale 1 the medians are 35866, 47982 and 89792 bytes (Pintado: 40322, 52754, 97564, 8 to 11 % higher;
indoor laboratory). The recorded sequence varies less than the model of the zed files assumed (CV 0.06
against 0.15) and more slowly (lag-1 autocorrelation 0.997 at 10 frames/s against 0.95). The sources
replay the sequence at the video rate of the run (4.3 to 42 outputs/s), so one output is one frame of the
sequence whatever the rate.

Stripe profile (`stripe-shares.json`, `make_stripe_profile.py`): the source replay of the stripe layers
gives stripe s of frame k the size round(F_k w_s), F_k the trace size of the frame; the sums of the shares
(expansion of a frame by 16 separate JPEG headers and DC restarts) are 1.18 (q90-s1), 1.28 (q70-s1),
1.36 (q50-s1), 1.59 (q90-s0p5), 1.88 (q70-s0p5) and 2.10 (q50-s0p5). The profile q90-s1 (stripes 0 to 7
above the horizon 0.039 to 0.053, stripes 8 to 15 0.086 to 0.112) is the default of dtp.json, whose frame
cap 98774 is the largest frame of `water-sbs-q90-s1.txt`, the default trace of the paper stack. The share
of a stripe varies over the pairs (CV 0.07 to 0.41, largest at the horizon); the replay keeps the mean
profile.

Proposed text for Section VI-C, Sources: "... one video topic of the kind output, whose output sizes replay
the JPEG sizes of a recorded stereo sequence over water (Pohang Canal Dataset [ref], CC BY-NC 4.0, 4500
frames) encoded as the side-by-side frames of the camera of the vessels, from a start that the seed fixes."

## Camera of Pintado (cross-check), how its traces were made, and their limitation

A recorded sequence of the camera was not available: the RP4 of Pintado (`pi@192.168.80.102`, reachable
from the emulation host) refused the keys of the emulation host (publickey, password), and the older
address 192.168.2.137 did not answer, so no frame was captured from `/dev/video*` (the task allowed a
read-only capture only).

1. **Still frames of the real camera.** The only frames of the ZED of Pintado at hand are one left and one
   right frame (HD720, 1280x720 per eye, PNG) that the hardware survey of Pintado captured on 2026-08-20
   (14:00 -03, file times; indoor laboratory bench, Stereolabs ZED serial 11104 on the Raspberry Pi 4 of
   the vessel, `README.md` of the survey directory): `zed_HD720_L.png` (1,026,391 bytes, MD5
   c0815bde644c651f97e81fea4a84c5a8) and `zed_HD720_R.png` (975,051 bytes, MD5
   88b74bfa0b088fb979813e7291f252b1), in `~/Documents/nsesd/lsa/projects/pintado-navio2/` on the
   emulation host (cassio-server). The images show people of the laboratory, so they stay out of the
   repository and out of the data archive; the archive holds the derived sizes (this directory). `make_trace.py` warps both eyes
   with the motion of a small vessel (roll 3 deg at a wave
   period of 3 s plus a random walk, pitch as a vertical shift, yaw as a random walk of the pan), crops
   the central 90 %, scales each eye to the WVGA eye (672x376) times S, adds an exposure drift (+-8 %)
   and fresh sensor noise (sigma 1.5), and encodes the side-by-side image with OpenCV 4.13.0 (libjpeg-turbo
   3.1.2, 4:2:0) at quality Q. The median sizes (bytes): Q50 40322 (S1), 15255 (S0.5); Q70 52754, 20012;
   Q90 97564, 35532. `python3 emu/traces/make_trace.py --left zed_HD720_L.png --right zed_HD720_R.png
   --out DIR` (seed 1) reproduces every file of this directory byte for byte (checked 2026-10-15).
   The scene does not change, so these sizes vary little (coefficient of variation 0.02).
2. **Model of the variation.** The traces of the experiments keep the median of step 1 and take the
   variation from a lognormal AR(1) model of the log size, sigma 0.15 and lag-1 correlation 0.95 per frame
   (realized: CV 0.12 to 0.17, lag-1 autocorrelation 0.93 to 0.96, maximum 1.4 to 1.9 times the median).
   These two parameters are assumed values for the MJPEG stream of a moving camera, not measurements.

These traces are not used by the experiments since 2026-10-15 (decision of the freeze): the experiments
replay the water sequence above, and the frames of Pintado (an indoor scene) serve as a cross-check of the
level of the sizes.
