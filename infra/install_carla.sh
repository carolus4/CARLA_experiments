#!/usr/bin/env bash
set -euo pipefail

# -----------------------------
# Config
# -----------------------------
CARLA_VERSION="${CARLA_VERSION:-0.9.16}"
CARLA_URL="${CARLA_URL:-https://tiny.carla.org/carla-0-9-16-linux}"

# Install location (override with CARLA_ROOT env)
# Prefer /workspace (persistent storage) if available, otherwise /opt
if [ -d "/workspace" ] && [ -w "/workspace" ]; then
  DEFAULT_CARLA_ROOT="/workspace/carla"
  DEFAULT_CARLA_CACHE="/workspace/carla_cache"
else
  DEFAULT_CARLA_ROOT="/opt/carla"
  DEFAULT_CARLA_CACHE="/opt/carla_cache"
fi
CARLA_ROOT="${CARLA_ROOT:-$DEFAULT_CARLA_ROOT}"
CARLA_TARBALL="${CARLA_TARBALL:-$DEFAULT_CARLA_CACHE/CARLA_${CARLA_VERSION}.tar.gz}"

# CARLA runtime user (CARLA refuses root)
CARLA_USER="${CARLA_USER:-carla}"

# Whether to chown the CARLA install dir to CARLA_USER
CHOWN_INSTALL="${CHOWN_INSTALL:-1}"

echo "==> CARLA install"
echo "    version:     $CARLA_VERSION"
echo "    url:         $CARLA_URL"
echo "    install dir: $CARLA_ROOT"
echo "    tarball:     $CARLA_TARBALL"
echo

if [ "$(id -u)" -ne 0 ]; then
  echo "⚠️ Please run as root (apt installs + user creation)."
  exit 1
fi

# -----------------------------
# System deps (runtime + tooling)
# -----------------------------
echo "==> Installing system dependencies..."
apt-get update -y

# Core runtime deps for CARLA headless + audio stubs + common libs
apt-get install -y --no-install-recommends \
  ca-certificates curl wget unzip \
  libvulkan1 vulkan-tools mesa-utils \
  libxrandr2 libxinerama1 libxcursor1 libxi6 libxkbcommon0 \
  libglib2.0-0 libsm6 libice6 libfontconfig1 libnss3 \
  libxdamage1 libxfixes3 libxrender1 libxext6 libx11-6 \
  libpulse0 libasound2 \
  ffmpeg \
  iproute2 \
  psmisc \
  openssh-client git \
  python3 python3-pip python3-venv

# If you’re using Xvfb for GL context stability, include it
apt-get install -y --no-install-recommends xvfb

# X11 socket dir (some headless paths expect it)
mkdir -p /tmp/.X11-unix
chmod 1777 /tmp/.X11-unix

echo "==> Dependencies installed."

# -----------------------------
# Check disk space
# -----------------------------
check_disk_space() {
  local path="$1"
  local required_gb="${2:-30}"
  local available_kb=$(df -k "$path" | tail -1 | awk '{print $4}')
  local available_gb=$((available_kb / 1024 / 1024))
  
  echo "==> Checking disk space for $path..."
  echo "    Available: ${available_gb}GB"
  echo "    Required:  ${required_gb}GB"
  
  if [ "$available_gb" -lt "$required_gb" ]; then
    echo "⚠️ ERROR: Insufficient disk space!"
    echo "    Need at least ${required_gb}GB free, but only ${available_gb}GB available on $(df -h "$path" | tail -1 | awk '{print $1}')"
    echo "    Consider:"
    echo "    1. Using persistent storage: CARLA_ROOT=/workspace/carla"
    echo "    2. Cleaning up old files: df -h"
    echo "    3. Increasing pod storage allocation"
    return 1
  fi
  return 0
}

# Check space for both cache (download) and install (extraction)
CACHE_DIR="$(dirname "$CARLA_TARBALL")"
INSTALL_DIR="$(dirname "$CARLA_ROOT")"

# Check cache directory (needs ~5GB for tarball)
if [ ! -f "$CARLA_TARBALL" ]; then
  if ! check_disk_space "$CACHE_DIR" 5; then
    exit 1
  fi
fi

# Check install directory (needs ~30GB for extracted CARLA)
if ! check_disk_space "$INSTALL_DIR" 30; then
  exit 1
fi

# -----------------------------
# Ensure install and cache directories
# -----------------------------
echo "==> Preparing install and cache directories..."
mkdir -p "$(dirname "$CARLA_TARBALL")"
mkdir -p "$CARLA_ROOT"

# -----------------------------
# Download CARLA tarball (cached)
# -----------------------------
if [ ! -f "$CARLA_TARBALL" ]; then
  echo "==> Downloading CARLA tarball to cache..."
  echo "    URL: $CARLA_URL"
  echo "    Destination: $CARLA_TARBALL"
  echo "    This may take 10-20 minutes depending on connection speed..."
  wget --progress=bar:force:noscroll -O "$CARLA_TARBALL" "$CARLA_URL"
  tarball_size=$(du -sh "$CARLA_TARBALL" 2>/dev/null | cut -f1)
  echo "==> Download complete. Size: $tarball_size"
else
  tarball_size=$(du -sh "$CARLA_TARBALL" 2>/dev/null | cut -f1)
  echo "==> Using cached tarball: $CARLA_TARBALL (Size: $tarball_size)"
fi

# -----------------------------
# Extract CARLA to CARLA_ROOT
# -----------------------------
# Marker file so we can rerun safely
MARKER="$CARLA_ROOT/.installed_${CARLA_VERSION}"

if [ -f "$MARKER" ]; then
  echo "==> CARLA already extracted for ${CARLA_VERSION} (marker exists). Skipping extract."
else
  echo "==> Extracting CARLA to $CARLA_ROOT ..."
  echo "    This may take 5-10 minutes for a ~20-30GB tarball..."
  rm -rf "$CARLA_ROOT"
  mkdir -p "$CARLA_ROOT"
  
  # Extract with verbose output and progress indicator
  # Use pv (pipe viewer) if available for progress, otherwise use tar -v for verbose
  if command -v pv >/dev/null 2>&1; then
    echo "    Using pv for progress indication..."
    pv "$CARLA_TARBALL" | tar -xz -C "$CARLA_ROOT" --no-same-owner
  else
    echo "    Extracting with verbose output (install 'pv' package for progress bar)..."
    # Use tar -v for verbose, and show periodic status updates in background
    # Track tar PID for better monitoring
    (
      TAR_PID=""
      while [ -z "$TAR_PID" ]; do
        sleep 1
        TAR_PID=$(pgrep -f "tar.*$CARLA_TARBALL" | head -1)
      done
      echo "    [Info] Tar process PID: $TAR_PID"
      last_size="0"
      stall_count=0
      while kill -0 "$TAR_PID" 2>/dev/null; do
        sleep 10
        if [ -d "$CARLA_ROOT" ]; then
          current_size=$(du -sm "$CARLA_ROOT" 2>/dev/null | cut -f1)
          extracted_size=$(du -sh "$CARLA_ROOT" 2>/dev/null | cut -f1)
          if [ "$current_size" = "$last_size" ]; then
            stall_count=$((stall_count + 1))
            if [ $stall_count -ge 6 ]; then
              echo "    ⚠️  WARNING: No progress for 60+ seconds. Extraction may be stuck."
              echo "    [Debug] Tar PID $TAR_PID still running. Current size: $extracted_size"
              echo "    [Debug] Check with: ps aux | grep $TAR_PID"
            fi
          else
            stall_count=0
          fi
          last_size="$current_size"
          echo "    [Progress] Extracted so far: $extracted_size ($current_size MB)"
        fi
      done
    ) &
    PROGRESS_PID=$!
    tar -xzvf "$CARLA_TARBALL" -C "$CARLA_ROOT" --no-same-owner
    EXTRACT_EXIT=$?
    kill $PROGRESS_PID 2>/dev/null || true
    wait $PROGRESS_PID 2>/dev/null || true
    
    if [ $EXTRACT_EXIT -ne 0 ]; then
      echo "⚠️  ERROR: Extraction failed with exit code $EXTRACT_EXIT"
      echo "    Check disk space: df -h $(dirname $CARLA_ROOT)"
      echo "    Check for errors: dmesg | tail -20"
      exit 1
    fi
  fi

  touch "$MARKER"
  final_size=$(du -sh "$CARLA_ROOT" 2>/dev/null | cut -f1)
  echo "==> Extract complete. Final size: $final_size"
fi

# Flatten helper: if CarlaUE4.sh is in a subdir, move its contents to CARLA_ROOT (never when it's already in CARLA_ROOT)
flatten_carla_root() {
  CARLA_SH=$(find "$CARLA_ROOT" -name "CarlaUE4.sh" -type f 2>/dev/null | head -1)
  [ -z "$CARLA_SH" ] && CARLA_SH=$(find "$CARLA_ROOT" -iname "carlaue4.sh" -type f 2>/dev/null | head -1)
  if [ -n "$CARLA_SH" ]; then
    SUBDIR=$(dirname "$CARLA_SH")
    # Already at CARLA_ROOT: nothing to flatten (tarball had no top-level dir). Do NOT rm -rf!
    SUBDIR_ABS=$(cd "$SUBDIR" 2>/dev/null && pwd -P)
    CARLA_ABS=$(cd "$CARLA_ROOT" 2>/dev/null && pwd -P)
    if [ -n "$SUBDIR_ABS" ] && [ "$SUBDIR_ABS" = "$CARLA_ABS" ]; then
      return 0
    fi
    echo "==> Flattening: $SUBDIR -> $CARLA_ROOT"
    mv "$SUBDIR"/* "$CARLA_ROOT/" 2>/dev/null || true
    for _f in "$SUBDIR"/.[!.]* "$SUBDIR"/..?*; do
      [ -e "$_f" ] && mv "$_f" "$CARLA_ROOT/" 2>/dev/null || true
    done
    rm -rf "$SUBDIR"
    find "$CARLA_ROOT" -depth -type d -empty -delete 2>/dev/null || true
    return 0
  fi
  return 1
}

# If CarlaUE4.sh not in CARLA_ROOT, try flattening (tarball has nested top-level dirs)
if [ ! -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
  flatten_carla_root || true
fi

# If still missing, layout may be corrupt (e.g. from old --strip-components extract). Re-extract.
if [ ! -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
  echo "==> CarlaUE4.sh not found under $CARLA_ROOT (corrupt layout). Re-extracting..."
  echo "    This may take 5-10 minutes..."
  rm -f "$MARKER"
  rm -rf "$CARLA_ROOT"
  mkdir -p "$CARLA_ROOT"
  
  # Use same progress indication as main extraction
  if command -v pv >/dev/null 2>&1; then
    pv "$CARLA_TARBALL" | tar -xz -C "$CARLA_ROOT" --no-same-owner
  else
    (
      TAR_PID=""
      while [ -z "$TAR_PID" ]; do
        sleep 1
        TAR_PID=$(pgrep -f "tar.*$CARLA_TARBALL" | head -1)
      done
      echo "    [Info] Tar process PID: $TAR_PID"
      last_size="0"
      stall_count=0
      while kill -0 "$TAR_PID" 2>/dev/null; do
        sleep 10
        if [ -d "$CARLA_ROOT" ]; then
          current_size=$(du -sm "$CARLA_ROOT" 2>/dev/null | cut -f1)
          extracted_size=$(du -sh "$CARLA_ROOT" 2>/dev/null | cut -f1)
          if [ "$current_size" = "$last_size" ]; then
            stall_count=$((stall_count + 1))
            if [ $stall_count -ge 6 ]; then
              echo "    ⚠️  WARNING: No progress for 60+ seconds. Extraction may be stuck."
            fi
          else
            stall_count=0
          fi
          last_size="$current_size"
          echo "    [Progress] Extracted so far: $extracted_size ($current_size MB)"
        fi
      done
    ) &
    PROGRESS_PID=$!
    tar -xzvf "$CARLA_TARBALL" -C "$CARLA_ROOT" --no-same-owner
    EXTRACT_EXIT=$?
    kill $PROGRESS_PID 2>/dev/null || true
    wait $PROGRESS_PID 2>/dev/null || true
    
    if [ $EXTRACT_EXIT -ne 0 ]; then
      echo "⚠️  ERROR: Re-extraction failed with exit code $EXTRACT_EXIT"
      exit 1
    fi
  fi
  
  touch "$MARKER"
  flatten_carla_root || true
fi

# Ensure executable
chmod +x "$CARLA_ROOT/CarlaUE4.sh" || true

# -----------------------------
# Create carla user
# -----------------------------
echo "==> Ensuring user '$CARLA_USER' exists..."
if ! id -u "$CARLA_USER" >/dev/null 2>&1; then
  useradd -m -s /bin/bash "$CARLA_USER"
  echo "    Created user '$CARLA_USER'"
else
  echo "    User '$CARLA_USER' already exists"
fi

# Optionally grant sudo (commented; enable if you want convenience)
# usermod -aG sudo "$CARLA_USER"

if [ "$CHOWN_INSTALL" = "1" ]; then
  echo "==> Setting ownership of $CARLA_ROOT to $CARLA_USER..."
  if chown -R "$CARLA_USER:$CARLA_USER" "$CARLA_ROOT" 2>/dev/null; then
    echo "    Ownership set successfully"
  else
    echo "    ⚠️  Warning: chown failed (may be due to filesystem restrictions)"
    echo "    Setting permissions instead to ensure $CARLA_USER can access files..."
    # Set permissions so carla user can read/execute (world-readable/executable)
    chmod -R u+rwX,go+rX "$CARLA_ROOT" 2>/dev/null || true
    # Ensure CarlaUE4.sh is executable
    find "$CARLA_ROOT" -name "CarlaUE4.sh" -type f -exec chmod +x {} \; 2>/dev/null || true
    echo "    Permissions set (chown skipped due to filesystem restrictions)"
  fi
fi

# -----------------------------
# Helper launch script
# -----------------------------
mkdir -p "$CARLA_ROOT/bin"
echo "==> Writing helper script to $CARLA_ROOT/bin/start_carla.sh ..."

cat > "$CARLA_ROOT/bin/start_carla.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

CARLA_ROOT="${CARLA_ROOT:-/opt/carla}"
CARLA_PORT="${CARLA_PORT:-2000}"

# Many headless GPU envs are more stable with OpenGL + Xvfb
USE_XVFB="${USE_XVFB:-1}"
CARLA_ARGS="${CARLA_ARGS:--RenderOffScreen -nosound -opengl -carla-rpc-port=${CARLA_PORT}}"

LOG="${LOG:-/home/carla/carla_${CARLA_PORT}.log}"

if [ ! -x "${CARLA_ROOT}/CarlaUE4.sh" ]; then
  echo "CarlaUE4.sh not found/executable at: ${CARLA_ROOT}/CarlaUE4.sh"
  exit 1
fi

# Start Xvfb if requested and not already running
if [ "$USE_XVFB" = "1" ]; then
  if ! pgrep -f "Xvfb :99" >/dev/null 2>&1; then
    nohup Xvfb :99 -screen 0 1920x1080x24 > /tmp/xvfb.log 2>&1 &
    sleep 0.5
  fi
  export DISPLAY=:99
fi

# Avoid multiple instances
if pgrep -u "$(id -u)" -f "CarlaUE4.sh" >/dev/null 2>&1; then
  echo "CARLA already running."
  exit 0
fi

cd "$CARLA_ROOT"
nohup ./CarlaUE4.sh $CARLA_ARGS > "$LOG" 2>&1 &
sleep 1

if pgrep -u "$(id -u)" -f "CarlaUE4.sh" >/dev/null 2>&1; then
  echo "CARLA started. Log: $LOG"
else
  echo "CARLA failed to start. Check log: $LOG"
  exit 1
fi
EOF

chmod +x "$CARLA_ROOT/bin/start_carla.sh"
if chown -R "$CARLA_USER:$CARLA_USER" "$CARLA_ROOT/bin" 2>/dev/null; then
  : # Ownership set successfully
else
  # If chown fails, ensure permissions are set correctly
  chmod -R u+rwX,go+rX "$CARLA_ROOT/bin" 2>/dev/null || true
fi

# -----------------------------
# Python client package (optional)
# -----------------------------
echo "==> Installing Python package carla==${CARLA_VERSION}..."
python3 -m pip install --upgrade pip
python3 -m pip install "carla==${CARLA_VERSION}"

# -----------------------------
# Project dependencies (if run from repo)
# -----------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
if [ -f "$REPO_ROOT/requirements.txt" ]; then
  echo "==> Installing project dependencies from requirements.txt..."
  python3 -m pip install -r "$REPO_ROOT/requirements.txt"
else
  echo "==> No requirements.txt at $REPO_ROOT; skipping project deps."
fi

echo
echo "==> Done."
echo "CARLA installed at: $CARLA_ROOT"
echo "Cached tarball at:  $CARLA_TARBALL"
echo

if [ "${START_CARLA:-0}" = "1" ]; then
  if [ ! -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
    echo "==> ERROR: CarlaUE4.sh missing at $CARLA_ROOT. Layout on pod:"
    ls -la "$CARLA_ROOT" 2>/dev/null || true
    echo "==> Any *Carla*/*.sh under CARLA_ROOT:"
    find "$CARLA_ROOT" -type f \( -name "*arla*" -o -name "*.sh" \) 2>/dev/null | head -30
    echo "==> Ensure repo on pod has latest install_carla.sh (git pull), then re-run install."
    exit 1
  fi
  echo "==> Starting CARLA (START_CARLA=1)..."
  su - "$CARLA_USER" -c "$CARLA_ROOT/bin/start_carla.sh"
else
  echo "Next:"
  echo "  su - $CARLA_USER"
  echo "  $CARLA_ROOT/bin/start_carla.sh"
fi