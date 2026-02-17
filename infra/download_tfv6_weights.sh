#!/usr/bin/env bash
set -euo pipefail

# Download TransfuserV6 pretrained weights to the persistent volume and symlink
# into the PCLA directory so the agent finds them at the expected path.
#
# Storage layout:
#   /workspace/pcla_weights/transfuserv6_pretrained/   ← actual files (persistent)
#   third_party/PCLA/pcla_agents/transfuserv6_pretrained  ← symlink to above
#
# Usage (on pod):
#   bash infra/download_tfv6_weights.sh
#
# Override defaults via env vars:
#   WEIGHTS_DIR=/my/path  PCLA_DIR=/my/pcla  bash infra/download_tfv6_weights.sh

# -----------------------------
# Config
# -----------------------------
WEIGHTS_DIR="${WEIGHTS_DIR:-/workspace/pcla_weights}"
WEIGHTS_URL="${WEIGHTS_URL:-https://huggingface.co/datasets/MasoudJTehrani/PCLA/resolve/main/pretrained.zip}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
PCLA_DIR="${PCLA_DIR:-$REPO_ROOT/third_party/PCLA}"

# What we need for tfv6_visiononly
EXPECTED_CONFIG="$WEIGHTS_DIR/transfuserv6_pretrained/visiononly_resnet34/config.json"

echo "==> TransfuserV6 weight download"
echo "    target dir: $WEIGHTS_DIR"
echo "    PCLA dir:   $PCLA_DIR"
echo

# -----------------------------
# Check if already downloaded
# -----------------------------
if [ -f "$EXPECTED_CONFIG" ]; then
    echo "==> Weights already present ($EXPECTED_CONFIG exists). Skipping download."
    echo "    To force re-download: rm -rf $WEIGHTS_DIR/transfuserv6_pretrained"
else
    # -----------------------------
    # Download
    # -----------------------------
    mkdir -p "$WEIGHTS_DIR"
    ZIP_PATH="$WEIGHTS_DIR/pretrained.zip"

    if [ -f "$ZIP_PATH" ]; then
        echo "==> Using cached zip: $ZIP_PATH"
    else
        echo "==> Downloading pretrained weights (~34 GB, this will take a while)..."
        echo "    URL: $WEIGHTS_URL"
        wget --progress=bar:force -O "$ZIP_PATH" "$WEIGHTS_URL"
    fi

    # -----------------------------
    # Extract only the TransfuserV6 files we need
    # -----------------------------
    echo "==> Extracting transfuserv6_pretrained/ from zip..."

    # List what's in the zip to find the right prefix
    # PCLA's pretrained.zip has pcla_agents/ at the top level
    # We extract only the transfuserv6_pretrained directory
    if unzip -l "$ZIP_PATH" | grep -q "pcla_agents/transfuserv6_pretrained/"; then
        # Extract with path rewriting: strip pcla_agents/ prefix
        cd "$WEIGHTS_DIR"
        unzip -o "$ZIP_PATH" "pcla_agents/transfuserv6_pretrained/*" -d "$WEIGHTS_DIR"
        # Move from pcla_agents/ to top level
        if [ -d "$WEIGHTS_DIR/pcla_agents/transfuserv6_pretrained" ]; then
            mv "$WEIGHTS_DIR/pcla_agents/transfuserv6_pretrained" "$WEIGHTS_DIR/transfuserv6_pretrained"
            rmdir "$WEIGHTS_DIR/pcla_agents" 2>/dev/null || true
        fi
    elif unzip -l "$ZIP_PATH" | grep -q "transfuserv6_pretrained/"; then
        # Already at top level in zip
        unzip -o "$ZIP_PATH" "transfuserv6_pretrained/*" -d "$WEIGHTS_DIR"
    else
        echo "WARNING: Could not find transfuserv6_pretrained/ in zip."
        echo "Extracting everything (will use more space)..."
        unzip -o "$ZIP_PATH" -d "$WEIGHTS_DIR"
    fi

    # Clean up zip to reclaim space (it's ~34 GB)
    echo "==> Removing zip to reclaim space..."
    rm -f "$ZIP_PATH"

    # Verify
    if [ ! -f "$EXPECTED_CONFIG" ]; then
        echo "ERROR: Expected config not found after extraction: $EXPECTED_CONFIG"
        echo "Contents of $WEIGHTS_DIR:"
        find "$WEIGHTS_DIR" -maxdepth 3 -type f | head -20
        exit 1
    fi
    echo "==> Weights extracted successfully."
fi

# -----------------------------
# Symlink into PCLA directory
# -----------------------------
PCLA_AGENTS_DIR="$PCLA_DIR/pcla_agents"
SYMLINK_TARGET="$PCLA_AGENTS_DIR/transfuserv6_pretrained"

if [ ! -d "$PCLA_DIR" ]; then
    echo "WARNING: PCLA directory not found at $PCLA_DIR."
    echo "Symlink not created. Initialize the submodule first:"
    echo "  cd $REPO_ROOT && git submodule update --init third_party/PCLA"
    echo
    echo "Then re-run this script, or manually symlink:"
    echo "  ln -sfn $WEIGHTS_DIR/transfuserv6_pretrained $SYMLINK_TARGET"
else
    mkdir -p "$PCLA_AGENTS_DIR"
    if [ -L "$SYMLINK_TARGET" ]; then
        echo "==> Symlink already exists: $SYMLINK_TARGET -> $(readlink "$SYMLINK_TARGET")"
    elif [ -d "$SYMLINK_TARGET" ]; then
        echo "==> $SYMLINK_TARGET is a real directory (not a symlink). Leaving as-is."
        echo "    If you want to use weights from $WEIGHTS_DIR, remove it and re-run."
    else
        ln -sfn "$WEIGHTS_DIR/transfuserv6_pretrained" "$SYMLINK_TARGET"
        echo "==> Created symlink: $SYMLINK_TARGET -> $WEIGHTS_DIR/transfuserv6_pretrained"
    fi
fi

echo
echo "==> Done."
echo "Weights location: $WEIGHTS_DIR/transfuserv6_pretrained/"
echo
echo "Verify with:"
echo "  ls $WEIGHTS_DIR/transfuserv6_pretrained/visiononly_resnet34/"
