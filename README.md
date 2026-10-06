# COSC471 TurtleBot3 simulation workspace

Shared between the Mac and the Ubuntu VM (UTM **VirtFS**, share tag `share`).

| Mac path | VM path |
|---|---|
| `.../COSC471-Robotics/vm-share` | `~/vm-share` (bindfs view of `/mnt/utm`) |

Code is edited on the Mac (by you or Claude); it is **built and run inside the VM**.

## Stack

Ubuntu 24.04 ARM64 → ROS 2 Jazzy Desktop → Gazebo Harmonic (`ros-jazzy-ros-gz`) → TurtleBot3 Burger → Python controller (`tb3_controller`).

## One-time setup (run inside the VM, in order)

Open **Terminal** in Ubuntu (Ctrl+Alt+T).

**Step 0: mount the share** (type these three lines by hand the first time):

```bash
sudo mkdir -p /mnt/utm
sudo mount -t 9p -o trans=virtio,version=9p2000.L share /mnt/utm
bash /mnt/utm/scripts/00-setup-share.sh
```

After this, the share mounts automatically at `~/vm-share` on every boot.

**Steps 1–3:**

```bash
bash ~/vm-share/scripts/01-install-ros-jazzy.sh           # 15-40 min, asks for your password
bash ~/vm-share/scripts/02-install-gazebo-turtlebot3.sh
bash ~/vm-share/scripts/03-build-workspace.sh
bash ~/vm-share/scripts/check-env.sh                       # every line should be [ OK ]
```

Close the terminal and open a new one after step 3 so `~/.bashrc` changes load.

## Daily use

| Goal | Command (inside VM) |
|---|---|
| Start simulator | `bash ~/vm-share/scripts/run-sim.sh` (add `--software` if the Gazebo window is black/crashes; add `house` or `empty` for other worlds) |
| Run the controller | second terminal: `bash ~/vm-share/scripts/run-controller.sh` |
| Both together | `ros2 launch tb3_controller sim_with_controller.launch.py` |
| Watch LiDAR sectors | `ros2 run tb3_controller scan_monitor --ros-args -p use_sim_time:=true` |
| Drive by keyboard | `bash ~/vm-share/scripts/run-teleop.sh` (stop the controller first) |
| Unit tests | `cd ~/vm-share/ros_ws/src/tb3_controller && python3 -m pytest -q test` |
| Tune while testing | `bash run-controller.sh -p max_linear:=0.12 -p stop_distance:=0.45` |

Editing `.py` files needs **no rebuild** (symlink install). Re-run `03-build-workspace.sh` after changing `setup.py`, `package.xml` or adding a launch file.

## Layout

```text
vm-share/
├── README.md            this file
├── explanation.md       how and why the controller works
├── scripts/             setup + run scripts (run inside VM)
└── ros_ws/src/tb3_controller/        ROS 2 ament_python package
    ├── tb3_controller/decision.py         pure decision logic (no ROS) - unit tested
    ├── tb3_controller/obstacle_avoider.py ROS node: /scan -> /cmd_vel
    ├── tb3_controller/scan_monitor.py     sensor check tool
    ├── launch/sim_with_controller.launch.py
    └── test/test_decision.py
```

Build output (`~/ros_ws/build`, `install`, `log`) lives on the VM disk, not in the share.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `mount: unknown filesystem type 9p` / no `share` device | UTM → VM settings → Sharing → Directory Share Mode = **VirtFS**, Path = vm-share; fully shut down and start the VM |
| Permission denied in `~/vm-share` | `ls -na /mnt/utm`; re-run `00-setup-share.sh` (it re-reads the Mac uid) |
| apt cannot reach servers | UTM Network mode **Emulated VLAN** (Shared Network NAT was blocked on this Mac by VPN software) |
| Gazebo window black/crash | `run-sim.sh --software`; UTM Display → `virtio-gpu-gl-pci` (GPU Supported); give the VM ≥4 CPU cores |
| Robot doesn't move | `ros2 topic type /cmd_vel`; if `geometry_msgs/msg/Twist` run with `-p cmd_vel_type:=unstamped`; make sure teleop isn't also running |
| `/scan` all `inf` | normal in `empty` world; use the default `world` |
