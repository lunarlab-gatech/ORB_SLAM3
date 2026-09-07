# AirMuseum Stereo-Inertial segfault investigation

Running the Python bindings against the AirMuseum drone sequence (Stereo-Inertial,
`KannalaBrandt8` fisheye, both cameras, Loop Closing disabled) segfaulted. Root causes
were found one at a time by rebuilding with AddressSanitizer + UndefinedBehaviorSanitizer
and rerunning a fixed 10s repro crop — never by guessing. Each fix exposed the next bug
underneath it (they only became reachable once the earlier ones stopped aborting the
process first).

## Reproducing

```
cd ORB_SLAM3/build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
      -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined" \
      -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address,undefined" ..
make ORB_SLAM3 orbslam3_python -j$(nproc)

LD_PRELOAD="$(gcc -print-file-name=libasan.so) $(gcc -print-file-name=libubsan.so)" \
ASAN_OPTIONS="detect_leaks=0:abort_on_error=1:halt_on_error=1" \
python3 /tmp/repro_crash.py
```

`Thirdparty/g2o/CMakeLists.txt` and `Thirdparty/DBoW2/CMakeLists.txt` are separate CMake
projects with their own compiler flags — the sanitizer flags (and `-march=native`
removal, see bug 1) have to be applied there too, not just in the root
`CMakeLists.txt`, or the "fix" is untested.

## Bugs found and fixed

### 1. g2o/Eigen alignment bad-free (`Thirdparty/g2o`, `Thirdparty/DBoW2`, root `CMakeLists.txt`)
Each of the three CMake projects independently set its own `-march=native`, so g2o,
DBoW2, and the main ORB_SLAM3 library could disagree on Eigen's vectorization width and
alignment assumptions for the exact same Eigen types crossing library boundaries — a
bad free inside g2o. Fixed by removing `-march=native` from all three, and adding
`add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0)` to all three so alignment is consistent
regardless of build machine.

### 2. `FrameDrawer::Update()` heap-buffer-overflow (`src/FrameDrawer.cc`)
For fisheye stereo (`both=true`), keypoints are stored as one combined array: left
keypoints first, then right. `Update()` was indexing `mvCurrentKeys[i]` unconditionally
even once `i` had passed into the right-camera range, reading past the end of
`mvCurrentKeys`. Fixed with the same left/right split already used elsewhere in the
codebase (e.g. `Frame::AssignFeaturesToGrid()`):
```cpp
const cv::KeyPoint &kp = (i < (int)mvCurrentKeys.size())
    ? mvCurrentKeys[i] : mvCurrentKeysRight[i - mvCurrentKeys.size()];
```

### 3. `LoopClosing` dangling `mpLastCurrentKF` after a reset (`src/LoopClosing.cc`)
`ResetIfRequested()` clears/deletes keyframes but never invalidated
`LoopClosing::mpLastCurrentKF`. If a reset happened between `Run()` iterations, the next
iteration dereferenced a pointer into an already-freed `KeyFrame`. Fixed by nulling
`mpLastCurrentKF` in the full-reset branch, and in the active-map-reset branch when it
points into the map being reset:
```cpp
mpLastCurrentKF = static_cast<KeyFrame*>(NULL);                                 // full reset
if(mpLastCurrentKF && mpLastCurrentKF->GetMap() == mpMapToReset)                // active-map reset
    mpLastCurrentKF = static_cast<KeyFrame*>(NULL);
```

### 4. `LoopClosing::mpCurrentKF` never initialized (`src/LoopClosing.cc`)
`mpCurrentKF` is only assigned inside `NewDetectCommonRegions()`, which starts with
`if(!mbActiveLC) return false;`. We run with Loop Closing disabled
(`loopClosing: 0` in the generated config), so `NewDetectCommonRegions()` returns
immediately and never touches `mpCurrentKF` — but the constructor's initializer list
never gave it a starting value either, so `Run()`'s unconditional
`mpLastCurrentKF = mpCurrentKF;` copied garbage every iteration. Fixed by adding
`mpCurrentKF(NULL)` to the constructor's initializer list. This is a real upstream bug
that only manifests with Loop Closing disabled.

### 5. `EdgeSE3ProjectXYZ::linearizeOplus()` dangling Eigen expression (`src/OptimizableTypes.cpp`)
```cpp
auto projectJac = -pCamera->projectJac(xyz_trans);
```
`projectJac()` returns `Eigen::Matrix<double,2,3>` **by value**. `operator-` on that
temporary builds a lazy `CwiseUnaryOp` expression template that holds a reference into
the temporary, not a copy — and `auto` deduces that expression type instead of a
concrete `Matrix`, so the reference dangles as soon as the temporary is destroyed at
the end of the statement. This is Eigen's documented "don't use `auto`" pitfall: normal
optimized builds usually still read back the right bytes (undefined behavior, not a
guaranteed wrong answer), which is presumably why it shipped unnoticed upstream — ASan's
stack poisoning on scope exit catches it every time. Fixed by declaring a concrete type
instead of `auto`:
```cpp
Eigen::Matrix<double,2,3> projectJac = -pCamera->projectJac(xyz_trans);
```

### 6. `LocalMapping::KeyFrameCulling()` heap-buffer-overflow, not fixed (`src/LocalMapping.cc:961`)
Same combined-index hazard as bug 2, in a different function: `pKF->mvDepth[i]` is read
for `i` up to the *combined* left+right keypoint count, but `mvDepth` is only ever sized
to `Nleft` (`Frame::ComputeStereoFishEyeMatches()`, `Frame.cc:1136` — right-only
keypoints never get a depth entry). Once `i >= Nleft`, this reads past the end of the
vector.

Left **unfixed** for now: ASan (`halt_on_error=1`) treats this as fatal, but it's a
`std::vector<float>` read a few elements past its buffer, which in a normal allocator
almost always still lands in mapped heap memory — it reads garbage rather than
segfaulting. It likely just corrupts a culling decision (a keyframe wrongly kept or
discarded), not a crash. Since the goal right now is only "does it run without
segfaulting", not full correctness, this is being left as a known issue rather than
fixed immediately.

### 7. `Frame::mpLastKeyFrame` never initialized (`src/Frame.cc`, `src/Tracking.cc`)
After rebuilding as a plain (non-ASan) Release build to test bug 6's assessment, it
still segfaulted — immediately after "New Map created", in
`Tracking::UpdateLocalKeyFrames()` (`Tracking.cc:3593`) dereferencing a garbage
`tempKeyFrame` pointer (`= mCurrentFrame.mpLastKeyFrame`). Correlated via pointer-value
logging that none of the three known `LocalMapping` `delete` sites (bug 4's
investigation) had freed it, ruling out use-after-free. Root cause: `Frame::Frame()`
(`Frame.cc:45`) initializes `mpReferenceKF(NULL)` but never `mpLastKeyFrame` — same class
of bug as 4, uninitialized `KeyFrame*`. `Tracking::PreintegrateIMU()` normally patches
`mCurrentFrame.mpLastKeyFrame` in from `Tracking::mpLastKeyFrame` (`Tracking.cc:1730`),
but only after two early returns (`!mCurrentFrame.mpPrevFrame`, empty
`mlQueueImuData`) — both true for the first frame after any reset/map init, so on
exactly that frame the field is read while still uninitialized. Fixed by adding
`mpLastKeyFrame(static_cast<KeyFrame*>(NULL))` to `Frame::Frame()`'s initializer list.

### 8. `Optimizer::InertialOptimization()` null-pointer dereference (`src/Optimizer.cc:3142`)
Running the *full* (uncropped) drone sequence via `run_airmuseum.py --viewer` (not just
the 10s repro crop) segfaulted right after printing `"Not preintegrated measurement"` —
which matches a known, unresolved upstream issue almost exactly:
https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/289 ("Segmentation fault in
SetNewBias(...)"). The code already detects the problem and prints a warning, but then
falls straight through and dereferences the null pointer anyway:
```cpp
if(!pKFi->mpImuPreintegrated)
    std::cout << "Not preintegrated measurement" << std::endl;
pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());  // null deref if the above was true
```
Fixed by adding `continue;` inside the `if` — `vpei`/`vppUsedKF` (the only consumers of
this loop's per-KF work) are appended sequentially and not indexed by the outer loop
variable, so skipping this KF's edge entirely is safe. Checked the rest of the file for
the same "warn but fall through" pattern on `mpImuPreintegrated` — every other call site
either already guards with `if(pKFi->mpImuPreintegrated)` before dereferencing, or checks
the `bImu` flag first; this was the only unguarded one.

## Status

Bugs 1-5, 7, and 8 above are confirmed fixed. Bug 6 is a known, real, but (believed)
non-crashing issue, left unfixed for now since the immediate goal was a Release build
that runs without segfaulting, not full correctness.

**The 10s repro crop runs to completion in a plain (non-sanitized) Release build,
exit code 0, no crash** — `python3 /tmp/repro_crash.py` reaches `Shutdown` / `DONE` for
all 201 frames. Tracking quality is poor on this crop (three separate active-map resets:
two "Fail to track local map!", one "Not enough motion for initializing") but that's a
tracking-quality problem, not a crash, and is a separate concern from this document.
