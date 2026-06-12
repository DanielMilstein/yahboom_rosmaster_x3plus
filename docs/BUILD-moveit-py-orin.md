# Building moveit_py from source on the Jetson Orin NX

The pick-place executor's `execute:=true` path imports `moveit.planning.MoveItPy`
(moveit_py, the MoveIt 2 Python bindings). moveit_py was never released as a
Humble **binary** — `ros-humble-moveit-py` does not exist in the apt repos —
but it *was* backported to the moveit2 repo's `humble` source branch. So any
machine that runs the executor with motion enabled needs MoveIt built from
source; dry runs (`execute:=false`) do not.

The sim machine's working setup (verified 2026-06-11) is the template:

- moveit2 cloned **inside the project workspace** at `src/moveit2`
  (untracked, alongside `src/yahboom_rosmaster`).
- Branch `humble`, commit `e7004f4b867351cd47f151efbd84693d6bcb15b8`
  ("Replace all moveit.ros.org to moveit.ai (#3675)").
- No sibling source repos — every MoveIt dependency satisfied by
  rosdep/apt binaries.

This recipe replicates that on the Orin. **Total time: 1–3 hours, most of it
unattended compilation.**

---

## Step 1 — Prepare the Orin

```bash
# Disk: need ~10 GB free for sources + build artifacts.
df -h ~

# Memory: compilation OOM is the #1 killer of MoveIt builds on Jetsons.
free -h
```

If swap is small or absent, add an 8 GB swapfile before building:

```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
# (add "/swapfile none swap sw 0 0" to /etc/fstab to persist, or skip if one-time)
```

Remove the binary MoveIt debs so the source build fully replaces them
(mixed apt/source MoveIt libraries can cause hard-to-debug ABI crashes):

```bash
sudo apt remove "ros-humble-moveit*"
```

**Review the removal list before confirming** — it should be moveit packages
only. Note: from this point until the build finishes,
`hardware_moveit.launch.py` will not run (move_group is gone). The bridge,
camera, and perception launches are unaffected.

## Step 2 — Clone moveit2 into the workspace, pinned to the sim commit

```bash
cd ~/yahboom_rosmaster_x3plus/src
git clone https://github.com/moveit/moveit2.git -b humble
git -C moveit2 checkout e7004f4b867351cd47f151efbd84693d6bcb15b8

cd ~/yahboom_rosmaster_x3plus
sudo apt update
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

A few rosdep keys occasionally fail on arm64 — `-r` makes rosdep continue
past them; deal with stragglers individually (usually `apt install
ros-humble-<dep>` or a missing system lib).

`src/moveit2` stays untracked in this repo, same as on the sim machine.

## Step 3 — Build (in tmux/screen — survive SSH drops)

```bash
cd ~/yahboom_rosmaster_x3plus
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j3 colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --parallel-workers 1
```

- `-DBUILD_TESTING=OFF` skips MoveIt's test suites — meaningfully faster on a
  Jetson, and the tests are also where apt/source library mixing first bites
  (undefined-reference link errors in `test_*` targets mean an apt moveit deb
  is still installed — see Step 1 and Troubleshooting).
- One workspace, one build: this compiles all of MoveIt *and* rebuilds the
  project packages against it. No underlay, no shell-rc changes — the usual
  `source install/setup.bash` now provides moveit_py too.
- `-j3` + one package at a time keeps peak memory inside what an Orin NX +
  swap can handle. If it still OOMs (compiler killed, board freezes), drop
  to `-j2`.
- Expect 1–3 hours. On any failure, fix the cause and **re-run the same
  command** — colcon skips packages that already built.
- Subsequent project-only rebuilds stay fast with
  `colcon build --base-paths src --packages-select <pkg> --symlink-install`.

## Step 4 — Verify

```bash
source install/setup.bash

# 1. The import that was failing:
python3 -c "from moveit.planning import MoveItPy; print('ok')"

# 2. The robot stack still comes up (move_group now from the source build):
ros2 launch x3plus_moveit_config hardware_moveit.launch.py moveit_rviz:=false

# 3. Executor dry run (perception + Gemini only, no motion):
ros2 launch gemini_pick_place_executor executor.launch.py \
  execute:=false use_gazebo:=false use_sim_time:=false

# 4. Only then: execute:=true.
```

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `c++: fatal error: Killed signal terminated program` | OOM — lower to `MAKEFLAGS=-j2`, confirm swap is active (`free -h`), re-run. |
| move_group segfaults or plugins fail to load after the build | An apt moveit deb survived and is mixing with the source build: `dpkg -l \| grep moveit` should show nothing; remove leftovers, delete `build/ install/` for moveit packages, rebuild. |
| rosdep can't resolve a key | Install that one dependency manually; `-r` already let the rest proceed. |
| `import moveit` works but executor fails on a missing symbol | moveit2 commit mismatch with the sim machine — must be `e7004f4b8` on `humble`. |
| Build is glacial | Confirm the Jetson is in MAXN power mode (`sudo nvpmodel -q`) and not thermally throttling. |
