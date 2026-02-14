#!/usr/bin/env bash
set -euo pipefail

# -----------------------------
# Config
# -----------------------------
CARLA_VERSION="${CARLA_VERSION:-0.9.16}"
CARLA_URL="${CARLA_URL:-https://tiny.carla.org/carla-0-9-16-linux}"

# Install location on the mounted volume (persists across pod migration)
PERSIST_ROOT="${PERSIST_ROOT:-/workspace}"
CARLA_ROOT="${CARLA_ROOT:-$PERSIST_ROOT/carla}"
CARLA_TARBALL="${CARLA_TARBALL:-$PERSIST_ROOT/cache/CARLA_${CARLA_VERSION}.tar.gz}"

# CARLA runtime user (CARLA refuses root)
CARLA_USER="${CARLA_USER:-carla}"

# Whether to chown the CARLA install dir to CARLA_USER
CHOWN_INSTALL="${CHOWN_INSTALL:-1}"

echo "==> CARLA persistent install"
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
# Ensure persistent directories
# -----------------------------
echo "==> Preparing persistent directories..."
mkdir -p "$PERSIST_ROOT/cache"
mkdir -p "$CARLA_ROOT"

# -----------------------------
# Download CARLA tarball (cached)
# -----------------------------
if [ ! -f "$CARLA_TARBALL" ]; then
  echo "==> Downloading CARLA tarball to cache..."
  wget -O "$CARLA_TARBALL" "$CARLA_URL"
else
  echo "==> Using cached tarball: $CARLA_TARBALL"
fi

# -----------------------------
# Extract CARLA into /workspace/carla
# -----------------------------
# Marker file so we can rerun safely
MARKER="$CARLA_ROOT/.installed_${CARLA_VERSION}"

if [ -f "$MARKER" ]; then
  echo "==> CARLA already extracted for ${CARLA_VERSION} (marker exists). Skipping extract."
else
  echo "==> Extracting CARLA to $CARLA_ROOT ..."
  # Clear target dir (avoid mixing versions)
  rm -rf "$CARLA_ROOT"
  mkdir -p "$CARLA_ROOT"
  tar -xzf "$CARLA_TARBALL" -C "$CARLA_ROOT" --strip-components=1

  touch "$MARKER"
  echo "==> Extract complete."
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
  chown -R "$CARLA_USER:$CARLA_USER" "$CARLA_ROOT"
fi

# -----------------------------
# Helper launch script (persistent)
# -----------------------------
echo "==> Writing helper scripts to $PERSIST_ROOT/bin ..."
mkdir -p "$PERSIST_ROOT/bin"

cat > "$PERSIST_ROOT/bin/start_carla.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

CARLA_ROOT="${CARLA_ROOT:-/workspace/carla}"
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

chmod +x "$PERSIST_ROOT/bin/start_carla.sh"
chown "$CARLA_USER:$CARLA_USER" "$PERSIST_ROOT/bin/start_carla.sh"

# -----------------------------
# Python client package (optional)
# -----------------------------
echo "==> Installing Python package carla==${CARLA_VERSION}..."
python3 -m pip install --upgrade pip
python3 -m pip install "carla==${CARLA_VERSION}"

echo
echo "==> Done."
echo "CARLA installed at: $CARLA_ROOT"
echo "Cached tarball at:  $CARLA_TARBALL"
echo
echo "Next:"
echo "  su - $CARLA_USER"
echo "  /workspace/bin/start_carla.sh"