#!/usr/bin/env bash
# setup-all.sh — one-shot provisioning of the Ubuntu 24.04 ARM64 VM for COSC471 Task 02.
# Order follows discuss/projects/robotics-project guides 03 -> 04 -> 05 -> 06 -> 07, plus VS Code.
# Asks for the sudo password ONCE (a keep-alive refreshes sudo while long apt steps run).
# Log: ~/vm-share/logs/setup-all.log
set -o pipefail
SHARE_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

[[ $EUID -ne 0 ]] || { echo "Run as your normal user."; exit 1; }
echo "sudo password is needed once for package installation:"
sudo -v || exit 1
( while true; do sudo -n true; sleep 45; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &
KEEPALIVE=$!
trap 'kill $KEEPALIVE 2>/dev/null' EXIT

# Wait for Ubuntu's background unattended-upgrades instead of failing on the dpkg lock.
echo 'DPkg::Lock::Timeout "900";' | sudo tee /etc/apt/apt.conf.d/99-lock-timeout >/dev/null

run() {  # run <name> <command...>; logs to ~/vm-share/logs once the share is mounted
  local name=$1; shift
  printf '\n\033[1;35m######## %s  (%s)\033[0m\n' "$name" "$(date '+%F %T')"
  "$@"; local rc=$?
  printf '######## %s finished rc=%s (%s)\n' "$name" "$rc" "$(date '+%F %T')"
  echo "$(date '+%F %T') $name rc=$rc" >> "$HOME/vm-share/logs/status.txt" 2>/dev/null || true
  return $rc
}

# ---------- Step 0: share (guide: UTM VirtFS) -------------------------------------------
run "00-setup-share" bash "$SHARE_SCRIPTS/00-setup-share.sh" || exit 1
mkdir -p "$HOME/vm-share/logs"
exec > >(tee -a "$HOME/vm-share/logs/setup-all.log") 2>&1
echo "$(date '+%F %T') setup-all started" >> "$HOME/vm-share/logs/status.txt"

# ---------- Guide 03: prepare Ubuntu (update, tools, locale, universe, suites) ----------
prep_ubuntu() {
  set -e
  export DEBIAN_FRONTEND=noninteractive
  sudo apt-get update
  sudo apt-get -y upgrade
  sudo apt-get install -y curl wget gpg ca-certificates git nano python3 python3-venv python3-pip \
      python3-pytest python3-matplotlib python3-numpy python3-pandas locales \
      software-properties-common mesa-utils spice-vdagent apt-transport-https
  locale | grep -qi utf-8 || { sudo locale-gen en_US en_US.UTF-8; sudo update-locale LANG=en_US.UTF-8; }
  sudo add-apt-repository -y universe
  echo "--- /etc/apt/sources.list.d/ubuntu.sources suites:"
  grep -E '^(URIs|Suites):' /etc/apt/sources.list.d/ubuntu.sources || true
  if ! grep -q 'noble-updates' /etc/apt/sources.list.d/ubuntu.sources; then
    echo "WARNING: noble-updates suite missing; ROS dev tools may conflict" ; fi
  echo "--- graphics (glxinfo -B):"
  glxinfo -B 2>/dev/null | grep -E 'renderer|version' || echo "(no DISPLAY for glxinfo)"
}
run "03-prepare-ubuntu" prep_ubuntu || exit 1

# ---------- Guide 04: ROS 2 Jazzy --------------------------------------------------------
run "04-ros2-jazzy" bash "$SHARE_SCRIPTS/01-install-ros-jazzy.sh" || exit 1

# ---------- Guides 05-06: Gazebo Harmonic + TurtleBot3 ----------------------------------
run "05-06-gazebo-tb3" bash "$SHARE_SCRIPTS/02-install-gazebo-turtlebot3.sh" || exit 1

# ---------- VS Code (official Microsoft apt repository, arm64/amd64) ---------------------
install_vscode() {
  set -e
  if command -v code >/dev/null; then echo "VS Code already installed: $(code --version | head -1)"; return; fi
  wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /tmp/microsoft.gpg
  sudo install -D -o root -g root -m 644 /tmp/microsoft.gpg /usr/share/keyrings/microsoft.gpg
  printf 'Types: deb\nURIs: https://packages.microsoft.com/repos/code\nSuites: stable\nComponents: main\nArchitectures: amd64,arm64,armhf\nSigned-By: /usr/share/keyrings/microsoft.gpg\n' \
    | sudo tee /etc/apt/sources.list.d/vscode.sources >/dev/null
  echo "code code/add-microsoft-repo boolean false" | sudo debconf-set-selections
  sudo apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y code
  code --version | head -1
  # Python + ROS friendly extensions (best effort; offline failure is non-fatal)
  code --install-extension ms-python.python --force || true
}
run "vscode" install_vscode || echo "VS Code install failed (non-fatal)"

# ---------- Guide 07: build workspace -----------------------------------------------------
run "07-build-workspace" bash "$SHARE_SCRIPTS/03-build-workspace.sh" || exit 1

run "check-env" bash "$SHARE_SCRIPTS/check-env.sh"
echo "$(date '+%F %T') SETUP-ALL DONE" >> "$HOME/vm-share/logs/status.txt"
printf '\n\033[1;32mSETUP-ALL DONE. Open a NEW terminal so ~/.bashrc changes load.\033[0m\n'
