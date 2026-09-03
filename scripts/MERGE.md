# Script merge candidates

Near-duplicate scripts in this directory that could be collapsed into a single
parameterized script. Diffed pairwise to confirm the only differences are
substituted names/paths, not logic.

## Done

- **#1** `extract_airmuseum_{drone,robotA,robotB,robotC}_depth_fast.py` (4 files)
  → merged into `extract_airmuseum_depth_fast.py --robot {drone,robotA,robotB,robotC}`.

- **#2** `run_airmuseum_{drone,robotA,robotB,robotC}_mono_depth{,_fast}.sh`
  (8 files — the drone variants were missed in the original scan) → merged
  into `run_airmuseum_mono_depth.sh --robot {drone,robotA,robotB,robotC}
  [--fast]`. The source `_fast` scripts printed a different "internal sanity
  check only, not part of the deliverable" message before the trajectory
  comparison; dropped per user request so all variants just say "Comparing
  trajectory...".

- **#3+#4+#5** `run_city_{drone1,drone2,husky1,husky2}_{with_lc,no_lc}.sh` (8),
  `run_aroundshort_{drone1,drone2,husky1,husky2}_{with_lc,no_lc}.sh` (8), and
  `run_citytest2_{drone2,husky1,husky2}_baseline{,_nolc}.sh` (5) — 21 files
  total → merged into `run_experiment.sh --scenario {city|aroundshort|citytest2}
  --robot ROBOT [--no-lc]`.

  Along the way, discovered the docker workflow these scripts assumed
  (ephemeral `docker run --rm` spinning up a fresh container per run, paths
  under `/workspace`, per-scenario host dataset mounts) is stale — the repo now
  uses a persistent container workflow (`docker/run_container.sh` +
  `enter_container.sh`) and expects the experiment script to already be
  running inside that container, with the repo checked out at its normal path
  and datasets under `~/data`. Rewrote `run_experiment.sh` accordingly:
  - no `docker run`/`docker exec` — the script assumes it's already running
    inside the container
  - dataset paths resolved under `~/data/<DATA_SUBDIR>` instead of a
    per-scenario host mount
  - deleted the now-obsolete `scripts/run_container_gui.sh` and
    `run_container_headless.sh` (superseded by `docker/run_container.sh` +
    `enter_container.sh`)

  Also standardized copy-paste drift within the merged 21 files: added the
  `if [ -f ..._traj_traj.txt ]` existence guard to the citytest2 scripts
  (which ran the comparison unconditionally); unified the pre-comparison echo
  to `"Comparing trajectory..."`; dropped the one-off `====` banner around
  `"Experiment complete!"` that only 2 of the 24 files had; dropped the
  hardcoded per-robot `"Frames: X to Ys"` comments (can't be derived
  generically, and the docker run's own log reports live progress); made the
  `"Loop Closure: ENABLED/DISABLED"` line consistent across all scenarios
  (previously only some citytest2 scripts had it).

  **NOTE:** `DATA_SUBDIR` per scenario (`raw_data_hercules/city_cslam_ugvuav_test1`,
  `raw_data_hercules/ausenv_aroundshortseq_2ugvuav`, `raw_data_hercules/city_test2`)
  is a best-effort guess carried over from the old per-scenario host paths —
  verify against the real `~/data` layout and fix if wrong.

- **#6** `convert_depth_robotA.py` / `convert_depth_airmuseum_robotB.py` /
  `convert_airmuseum_depth_robotC.py` (3 files) → merged into
  `convert_depth_airmuseum.py --robot {drone,robotA,robotB,robotC}`. Used
  robotB's version as the base (most robust: nearest-neighbor via a
  candidates list rather than robotA's clip-based search, plus
  `isfinite`/negative-depth clamping that robotA and robotC didn't both have),
  folded robotA's extra match-gap stats (median, >50ms count) back in.
  `drone` added to `--robot` on request even though no drone-specific script
  existed in this set (only the Fast-FoundationStereo pipeline, #1, had one) —
  paths are already parameterized generically so no template change was needed.

## Not worth merging

- `monitor_*.sh` (8 files) — each hardcodes different log filenames,
  backup-file globs, and ad hoc progress math for whatever experiment was
  running that day. Read as disposable one-off monitors rather than a
  stable interface.

- `extract_airmuseum_data.py` / `extract_airmuseum_data_robotB.py` /
  `extract_airmuseum_robotA_data.py` / `extract_airmuseum_robotC.py` — share
  boilerplate but genuinely branch on IMU-vs-no-IMU and
  stereo-inertial-vs-mono-depth, so a merge would need a real mode flag, not
  just a name swap.

- `convert_depth.py` vs `convert_depth_robotA.py` — not real duplicates,
  different algorithms (old naive vs nearest-timestamp-match).

## Directory layout (done)

`scripts/` was ~40 flat files with no grouping. Split by function into
subdirectories:

- **`extract/`** — pulls raw bag data into this workspace's flat-file format
  - `extract_airmuseum_data.py`, `extract_airmuseum_data_robotB.py`,
    `extract_airmuseum_robotA_data.py`, `extract_airmuseum_robotC.py`

- **`convert/`** — depth `.npy` → 16-bit PNG conversion
  - `convert_depth.py`, `convert_depth_airmuseum.py`,
    `extract_airmuseum_depth_fast.py` (named "extract" but functionally a
    depth converter like the other two — grouping by behavior, not by name)

- **`run/`** — experiment runners (docker/SLAM invocations)
  - `run_experiment.sh`, `run_airmuseum_mono_depth.sh`,
    `run_airmuseum_drone_stereo_inertial.sh`, `run_around_all.sh`,
    `run_forest_all.sh`, `run_drone1_rgbd_inertial.sh`,
    `run_drone1_no_loop_closure.sh`, `run_drone1_no_loop_closure_trimmed.sh`,
    `run_drone2_no_loop_closure.sh`, `run_husky1_no_lc.sh`,
    `run_husky1_with_lc.sh`, `run_husky2_no_lc.sh`, `run_husky2_with_lc.sh`,
    `run_city_drone2_baseline_trimmed.sh`, `run_city_drone2_conservative_lc.sh`,
    `run_city_husky1_baseline_trimmed.sh`, `run_city_husky1_fastmotion.sh`,
    `run_city_husky1_fastmotion_nolc.sh`, `run_city_husky2_baseline_trimmed.sh`,
    `run_city_husky2_fastmotion.sh`, `run_city_husky2_fastmotion_nolc.sh`

- **`monitor/`** — polling scripts that tail logs/backups for a running experiment
  - `auto_monitor_and_analyze.sh`, `monitor_all.sh`, `monitor_current.sh`,
    `monitor_drone1_nolc.sh`, `monitor_drone1_nolc_trimmed.sh`,
    `monitor_drone2.sh`, `monitor_drone2_nolc.sh`, `monitor_imu_trimmed.sh`,
    `monitor_stereo.sh`

- **`analyze/`** — post-hoc trajectory analysis/comparison
  - `analyze_all_results.py`, `compare_trajectory.py`,
    `compare_trajectory_simple.py`

- **`setup/`** — one-off dataset/calibration prep utilities, not tied to a
  specific experiment run
  - `cleanup_dataset_configs.sh`, `compute_camera_imu_transform.py`,
    `fix_imu_generation.py`, `gravity_align_airmuseum.py`

Cross-references fixed as part of the move: every `run/*.sh` and `monitor/*.sh`
script that called `compare_trajectory_simple.py` (bare, assuming cwd) now
resolves it relative to its own location —
`"$(dirname "$0")/../analyze/compare_trajectory_simple.py"`
(`run_experiment.sh` already had a `$SCRIPT_DIR` var, updated to
`$SCRIPT_DIR/../analyze/compare_trajectory_simple.py`). `monitor_stereo.sh`
had one occurrence inside an *echoed instructional string* (not a real
invocation) — fixed to a plain `scripts/analyze/compare_trajectory_simple.py`
since that line assumes the user is at the repo root, not at the script's own
directory. Also updated the three stale `scripts/...` references in
`README.md` to their new subdirectory paths. All moved/edited `.sh` files pass
`bash -n`.
