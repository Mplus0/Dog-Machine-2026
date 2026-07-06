#!/usr/bin/env bash
set -u

SCRIPT_NAME="$(basename "$0")"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEFAULT_OUT="${HOME}/robot_env_snapshots/env_${STAMP}"
OUT_DIR="${1:-$DEFAULT_OUT}"

mkdir -p "$OUT_DIR"

LOG_FILE="${OUT_DIR}/_commands.log"
SUMMARY_FILE="${OUT_DIR}/SUMMARY.txt"
COMBINED_FILE="${OUT_DIR}.txt"

log() {
  printf '[%s] %s\n' "$(date '+%F %T')" "$*" | tee -a "$LOG_FILE" >/dev/null
}

run_cmd() {
  local name="$1"
  shift
  local file="${OUT_DIR}/${name}.txt"
  log "RUN ${name}: $*"
  {
    printf '$ %s\n\n' "$*"
    "$@"
    local code=$?
    printf '\n[exit_code] %s\n' "$code"
  } >"$file" 2>&1
}

run_shell() {
  local name="$1"
  local cmd="$2"
  local file="${OUT_DIR}/${name}.txt"
  log "RUN ${name}: ${cmd}"
  {
    printf '$ %s\n\n' "$cmd"
    bash -lc "$cmd"
    local code=$?
    printf '\n[exit_code] %s\n' "$code"
  } >"$file" 2>&1
}

run_python() {
  local name="$1"
  local code="$2"
  local file="${OUT_DIR}/${name}.txt"
  log "RUN ${name}: python3 -"
  {
    printf '$ python3 - <<PY\n%s\nPY\n\n' "$code"
    printf '%s\n' "$code" | python3 -
    local code_status=$?
    printf '\n[exit_code] %s\n' "$code_status"
  } >"$file" 2>&1
}

copy_if_exists() {
  local src="$1"
  local dst_name="$2"
  if [ -e "$src" ]; then
    log "COPY ${src}"
    cp -a "$src" "${OUT_DIR}/${dst_name}" 2>>"$LOG_FILE" || true
  fi
}

cat >"$SUMMARY_FILE" <<EOF
Robot environment snapshot
stamp: ${STAMP}
host: $(hostname 2>/dev/null || true)
user: $(id -un 2>/dev/null || true)
output_dir: ${OUT_DIR}
script: ${SCRIPT_NAME}

This script is read-only. It does not install, upgrade, remove, or modify packages.
EOF

run_cmd "date" date --iso-8601=seconds
run_cmd "uname" uname -a
run_cmd "hostnamectl" hostnamectl
run_shell "os_release" "cat /etc/os-release; echo; lsb_release -a 2>/dev/null || true"
run_cmd "whoami_id" id
run_cmd "uptime" uptime
run_cmd "df_h" df -h
run_cmd "free_h" free -h
run_cmd "lsblk" lsblk -o NAME,MODEL,SIZE,TYPE,FSTYPE,MOUNTPOINT
run_shell "env_filtered" "env | sort | grep -E '^(ROS|CUDA|CUDNN|TENSORRT|LD_LIBRARY_PATH|PATH|PYTHONPATH|CONDA|VIRTUAL_ENV|CATKIN|CMAKE|PKG_CONFIG|DISPLAY|XDG)=' || true"

run_shell "jetson_l4t" "cat /etc/nv_tegra_release 2>/dev/null || true; echo; dpkg-query -W 'nvidia-l4t-*' 2>/dev/null | sort || true"
run_shell "jetson_tools" "which jetson_release 2>/dev/null && jetson_release || true; echo; which tegrastats 2>/dev/null || true; which jtop 2>/dev/null || true"
run_shell "cuda" "which nvcc 2>/dev/null || true; nvcc --version 2>/dev/null || true; echo; ls -ld /usr/local/cuda* 2>/dev/null || true; echo; dpkg-query -W '*cuda*' '*cudnn*' '*tensorrt*' '*nvinfer*' 2>/dev/null | sort || true"
run_shell "gpu_runtime" "nvidia-smi 2>/dev/null || true; echo; ldconfig -p 2>/dev/null | grep -Ei 'cuda|cudnn|nvinfer|tensorrt' | sort || true"

run_shell "ros_env" "echo ROS_DISTRO=\$ROS_DISTRO; echo ROS_VERSION=\$ROS_VERSION; echo ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH; echo CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH; echo; which roscore 2>/dev/null || true; rosversion -d 2>/dev/null || true; rosversion ros_comm 2>/dev/null || true"
run_shell "ros_tools" "which catkin_make 2>/dev/null || true; which catkin 2>/dev/null || true; which rospack 2>/dev/null || true; which rosdep 2>/dev/null || true; catkin_make --version 2>/dev/null || true; catkin --version 2>/dev/null || true; rosdep --version 2>/dev/null || true"
run_shell "ros_packages_core" "dpkg-query -W 'ros-*' 2>/dev/null | sort || true"
run_shell "ros_workspace_packages" "if command -v rospack >/dev/null 2>&1; then rospack list 2>/dev/null | sort; fi"
run_shell "ros_params_if_master" "if command -v rosparam >/dev/null 2>&1 && timeout 2s rosparam list >/dev/null 2>&1; then rosparam list | sort; else echo 'ROS master not reachable'; fi"
run_shell "ros_topics_if_master" "if command -v rostopic >/dev/null 2>&1 && timeout 2s rostopic list >/dev/null 2>&1; then rostopic list | sort; else echo 'ROS master not reachable'; fi"

run_shell "python_versions" "which python 2>/dev/null || true; python --version 2>&1 || true; which python3 2>/dev/null || true; python3 --version 2>&1 || true; which pip 2>/dev/null || true; pip --version 2>/dev/null || true; which pip3 2>/dev/null || true; pip3 --version 2>/dev/null || true"
run_python "python3_sysconfig" "import sys, sysconfig, site, platform
print('executable:', sys.executable)
print('version:', sys.version)
print('platform:', platform.platform())
print('prefix:', sys.prefix)
print('base_prefix:', getattr(sys, 'base_prefix', ''))
print('paths:')
for p in sys.path:
    print(' ', p)
print('sitepackages:', site.getsitepackages() if hasattr(site, 'getsitepackages') else '')
print('usersite:', site.getusersitepackages())
print('config vars:')
for k in ['CC','CXX','SOABI','INCLUDEPY','LIBDIR']:
    print(k, sysconfig.get_config_var(k))"
run_python "python3_import_versions" "mods = ['rospy','cv2','numpy','scipy','yaml','PIL','matplotlib','torch','torchvision','onnx','onnxruntime','tensorrt','pycuda','pyrealsense2','ultralytics','serial','cv_bridge']
for name in mods:
    try:
        mod = __import__(name)
        version = getattr(mod, '__version__', '')
        path = getattr(mod, '__file__', '')
        print(f'{name}: OK version={version} file={path}')
    except Exception as exc:
        print(f'{name}: FAIL {type(exc).__name__}: {exc}')"
run_shell "pip3_list" "python3 -m pip list 2>/dev/null || pip3 list 2>/dev/null || true"
run_shell "pip3_freeze" "python3 -m pip freeze 2>/dev/null || pip3 freeze 2>/dev/null || true"

run_shell "realsense_tools" "which rs-enumerate-devices 2>/dev/null || true; which realsense-viewer 2>/dev/null || true; rs-enumerate-devices --version 2>/dev/null || true; echo; dpkg-query -W '*realsense*' '*librealsense*' 2>/dev/null | sort || true"
run_shell "realsense_devices" "rs-enumerate-devices -s 2>/dev/null || true; echo; rs-enumerate-devices -c 2>/dev/null || true"
run_shell "usb_video_devices" "lsusb 2>/dev/null || true; echo; v4l2-ctl --list-devices 2>/dev/null || true; echo; ls -l /dev/video* /dev/realsense* 2>/dev/null || true"
run_shell "udev_realsense" "ls -l /etc/udev/rules.d/*realsense* /lib/udev/rules.d/*realsense* 2>/dev/null || true"

run_shell "toolchain_versions" "gcc --version 2>/dev/null | head -n 1 || true; g++ --version 2>/dev/null | head -n 1 || true; cmake --version 2>/dev/null | head -n 1 || true; make --version 2>/dev/null | head -n 1 || true; ninja --version 2>/dev/null || true; pkg-config --version 2>/dev/null || true; git --version 2>/dev/null || true"
run_shell "native_libs_versions" "dpkg-query -W 'build-essential' 'cmake' 'gcc*' 'g++*' 'libboost*' 'libeigen3-dev' 'libopencv*' 'libpcl*' 'libceres*' 'libsuitesparse*' 'libyaml-cpp*' 'libgflags*' 'libgoogle-glog*' 'libgl*' 'libglew*' 'libglfw*' 2>/dev/null | sort || true"
run_shell "pkg_config_relevant" "pkg-config --list-all 2>/dev/null | grep -Ei 'opencv|eigen|ceres|pcl|yaml|boost|realsense|cuda|glog|gflags|sophus|pangolin|opencv' | sort || true"
run_python "opencv_build_info" "try:
    import cv2
    print(cv2.__version__)
    print(cv2.getBuildInformation())
except Exception as exc:
    print('cv2 unavailable:', exc)"
run_shell "ldconfig_relevant" "ldconfig -p 2>/dev/null | grep -Ei 'opencv|pcl|ceres|suitesparse|yaml-cpp|boost|realsense|glog|gflags|pangolin|sophus|cuda|cudnn|nvinfer' | sort || true"

run_shell "docker" "which docker 2>/dev/null || true; docker --version 2>/dev/null || true; docker info 2>/dev/null || true; docker images 2>/dev/null || true; docker ps -a 2>/dev/null || true"
run_shell "network_basic" "ip addr; echo; ip route; echo; ip neigh"
run_shell "services_relevant" "systemctl --no-pager --type=service --state=running 2>/dev/null | grep -Ei 'docker|network|ssh|ros|realsense|nv|nvidia' || true"
run_shell "apt_sources" "grep -Rhs '^deb ' /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null | sort || true"
run_shell "apt_installed_all" "dpkg-query -W 2>/dev/null | sort || true"

copy_if_exists "/etc/nv_tegra_release" "nv_tegra_release"
copy_if_exists "/etc/apt/sources.list" "apt_sources.list"

tar_path="${OUT_DIR}.tar.gz"
tar -czf "$tar_path" -C "$(dirname "$OUT_DIR")" "$(basename "$OUT_DIR")" 2>>"$LOG_FILE" || true

{
  printf 'Robot environment snapshot combined report\n'
  printf 'stamp: %s\n' "$STAMP"
  printf 'directory: %s\n' "$OUT_DIR"
  printf 'archive: %s\n' "$tar_path"
  printf '\n'
  for file in "$OUT_DIR"/*.txt; do
    [ -f "$file" ] || continue
    printf '\n'
    printf '================================================================================\n'
    printf 'FILE: %s\n' "$(basename "$file")"
    printf '================================================================================\n'
    cat "$file"
    printf '\n'
  done
} >"$COMBINED_FILE" 2>&1

cat <<EOF
Environment snapshot finished.
Directory: ${OUT_DIR}
Archive:   ${tar_path}
Combined:  ${COMBINED_FILE}

Send back the combined txt for quick copy, or the tar.gz archive for full analysis.
EOF
