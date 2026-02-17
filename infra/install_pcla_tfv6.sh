#!/usr/bin/env bash
set -euo pipefail

# Install PCLA TransfuserV6 (visiononly) dependencies into a persistent venv.
#
# Designed for RunPod (or similar) where:
#   /workspace  = persistent volume (60-80 GB)   ← venv, pip cache, weights go here
#   /opt        = ephemeral (holds CARLA binary)  ← do NOT put PyTorch here
#
# Usage (as root on pod):
#   bash infra/install_pcla_tfv6.sh
#
# After this, activate with:
#   source /workspace/venvs/pcla/bin/activate

# -----------------------------
# Config
# -----------------------------
VENV_DIR="${VENV_DIR:-/workspace/venvs/pcla}"
PIP_CACHE="${PIP_CACHE:-/workspace/.pip-cache}"

# Python version: prefer 3.8 if available, fall back to system python3
PYTHON="${PYTHON:-}"

# Where the repo lives on the pod
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
REQUIREMENTS="$REPO_ROOT/requirements-pcla-tfv6.txt"

echo "==> PCLA TransfuserV6 venv install"
echo "    venv:         $VENV_DIR"
echo "    pip cache:    $PIP_CACHE"
echo "    requirements: $REQUIREMENTS"
echo

# -----------------------------
# Find a suitable Python
# -----------------------------
find_python() {
    if [ -n "$PYTHON" ] && command -v "$PYTHON" >/dev/null 2>&1; then
        echo "$PYTHON"
        return
    fi
    # Prefer 3.8 for PCLA compatibility
    for candidate in python3.8 python3 python; do
        if command -v "$candidate" >/dev/null 2>&1; then
            echo "$candidate"
            return
        fi
    done
    echo ""
}

PYTHON_BIN="$(find_python)"
if [ -z "$PYTHON_BIN" ]; then
    echo "ERROR: No Python found. Install python3 first."
    exit 1
fi

PY_VERSION=$("$PYTHON_BIN" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "==> Using Python: $PYTHON_BIN ($PY_VERSION)"

if [ "$PY_VERSION" != "3.8" ]; then
    echo "    WARNING: PCLA was built against Python 3.8. Using $PY_VERSION — some packages may"
    echo "    have compatibility issues. If you hit problems, install python3.8 via deadsnakes PPA:"
    echo "      apt install software-properties-common && add-apt-repository ppa:deadsnakes/ppa"
    echo "      apt install python3.8 python3.8-venv python3.8-dev"
    echo "      PYTHON=python3.8 bash infra/install_pcla_tfv6.sh"
    echo
fi

# -----------------------------
# Create venv on persistent volume
# -----------------------------
mkdir -p "$(dirname "$VENV_DIR")"
mkdir -p "$PIP_CACHE"

if [ -d "$VENV_DIR/bin" ]; then
    echo "==> Venv already exists at $VENV_DIR. Reusing."
else
    echo "==> Creating venv at $VENV_DIR ..."
    "$PYTHON_BIN" -m venv "$VENV_DIR"
fi

# Activate
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

# Ensure pip is up to date
pip install --cache-dir "$PIP_CACHE" --upgrade pip setuptools wheel

# -----------------------------
# Detect CUDA version for PyTorch
# -----------------------------
detect_cuda_version() {
    # Try nvcc first
    if command -v nvcc >/dev/null 2>&1; then
        nvcc --version 2>/dev/null | grep -oP 'release \K[0-9]+\.[0-9]+' | head -1
        return
    fi
    # Fall back to nvidia-smi
    if command -v nvidia-smi >/dev/null 2>&1; then
        nvidia-smi 2>/dev/null | grep -oP 'CUDA Version: \K[0-9]+\.[0-9]+' | head -1
        return
    fi
    echo ""
}

CUDA_VER="$(detect_cuda_version)"
echo "==> Detected CUDA: ${CUDA_VER:-none}"

# Map CUDA version to PyTorch index URL
# Using PyTorch 2.1.x for broad compatibility (supports cu118, cu121)
if [ -z "$CUDA_VER" ]; then
    echo "WARNING: No CUDA detected. Installing CPU-only PyTorch (GPU inference will NOT work)."
    TORCH_INDEX="https://download.pytorch.org/whl/cpu"
    TORCH_CU_TAG="cpu"
elif echo "$CUDA_VER" | grep -qE "^12\." ; then
    TORCH_INDEX="https://download.pytorch.org/whl/cu121"
    TORCH_CU_TAG="cu121"
elif echo "$CUDA_VER" | grep -qE "^11\." ; then
    TORCH_INDEX="https://download.pytorch.org/whl/cu118"
    TORCH_CU_TAG="cu118"
else
    echo "WARNING: Unrecognised CUDA $CUDA_VER. Trying cu121."
    TORCH_INDEX="https://download.pytorch.org/whl/cu121"
    TORCH_CU_TAG="cu121"
fi

echo "==> PyTorch index: $TORCH_INDEX (tag: $TORCH_CU_TAG)"

# -----------------------------
# Install PyTorch + torchvision
# -----------------------------
echo "==> Installing PyTorch + torchvision ..."
pip install --cache-dir "$PIP_CACHE" \
    torch torchvision \
    --index-url "$TORCH_INDEX"

# Verify torch is importable and CUDA-aware
python -c "
import torch
print(f'  torch {torch.__version__}  CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'  GPU: {torch.cuda.get_device_name(0)}')
"

# -----------------------------
# Install CARLA Python client
# -----------------------------
echo "==> Installing carla==0.9.16 ..."
pip install --cache-dir "$PIP_CACHE" "carla==0.9.16" || {
    echo "WARNING: pip install carla==0.9.16 failed. Trying from PCLA dist/ if available..."
    PCLA_DIR="$REPO_ROOT/third_party/PCLA"
    CARLA_WHL=$(find "$PCLA_DIR/dist" -name "carla-0.9.16*.whl" 2>/dev/null | head -1)
    if [ -n "$CARLA_WHL" ]; then
        pip install --cache-dir "$PIP_CACHE" "$CARLA_WHL"
    else
        echo "ERROR: Cannot install carla==0.9.16. Install it manually."
        exit 1
    fi
}

# -----------------------------
# Install remaining deps from requirements file
# -----------------------------
if [ ! -f "$REQUIREMENTS" ]; then
    echo "ERROR: Requirements file not found: $REQUIREMENTS"
    exit 1
fi

echo "==> Installing dependencies from $REQUIREMENTS ..."
pip install --cache-dir "$PIP_CACHE" -r "$REQUIREMENTS"

# -----------------------------
# Install torch-scatter (must come AFTER torch)
# -----------------------------
echo "==> Installing torch-scatter ..."

# Try prebuilt wheel first (fastest, avoids build issues)
TORCH_VER=$(python -c "import torch; print(torch.__version__.split('+')[0])")
SCATTER_WHL_URL="https://data.pyg.org/whl/torch-${TORCH_VER}+${TORCH_CU_TAG}.html"

echo "    Trying prebuilt wheel from $SCATTER_WHL_URL ..."
if pip install --cache-dir "$PIP_CACHE" torch-scatter -f "$SCATTER_WHL_URL" 2>/dev/null; then
    echo "    torch-scatter installed from prebuilt wheel."
else
    echo "    Prebuilt wheel not found. Building from source (may take a few minutes)..."
    pip install --cache-dir "$PIP_CACHE" --no-build-isolation torch-scatter
fi

# -----------------------------
# Verify key imports
# -----------------------------
echo "==> Verifying key imports..."
python -c "
import torch, torchvision, timm, carla
print(f'  torch={torch.__version__}  torchvision={torchvision.__version__}')
print(f'  timm={timm.__version__}  carla={carla.__version__ if hasattr(carla, \"__version__\") else \"ok\"}')
import torch_scatter
print(f'  torch_scatter ok')
print('  All key imports successful.')
"

echo
echo "==> Done."
echo "Activate the venv with:"
echo "  source $VENV_DIR/bin/activate"
echo
echo "Next: download TransfuserV6 weights with:"
echo "  bash infra/download_tfv6_weights.sh"
