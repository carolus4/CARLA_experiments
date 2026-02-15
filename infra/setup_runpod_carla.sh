#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage:"
  echo "  ./setup_runpod_carla.sh \"ssh user@host -p 12345 -i ~/.ssh/id_ed25519\" [--no-install]"
  echo ""
  echo "  By default: installs CARLA on the pod if missing, then starts CARLA (ready to develop)."
  echo "  --no-install  Skip install; only start CARLA if already installed."
  exit 1
fi

SSH_CMD="$1"
INSTALL=1
[ "${2:-}" = "--no-install" ] && INSTALL=0

# -----------------------------
# Config (override via env vars)
# -----------------------------
ALIAS="${ALIAS:-runpod-carla}"

# Where your code lives on the pod
REPO_DIR="${REPO_DIR:-/workspace/CARLA_experiments}"
# Optional: if REPO_DIR doesn't exist, provide REPO_URL to clone
REPO_URL="${REPO_URL:-}"

# Where CARLA is installed on the pod
CARLA_DIR="${CARLA_DIR:-/opt/carla}"
CARLA_PORT="${CARLA_PORT:-2000}"

# Additional CARLA args (you can override if needed)
CARLA_ARGS="${CARLA_ARGS:--RenderOffScreen -nosound -carla-rpc-port=${CARLA_PORT}}"

CONFIG="$HOME/.ssh/config"

# GitHub deploy key on your Mac (private key). Required for git@github.com pulls as carla.
GITHUB_DEPLOY_KEY="${GITHUB_DEPLOY_KEY:-$HOME/.ssh/id_ed25519_carla_deploy}"
GITHUB_DEPLOY_KEY="${GITHUB_DEPLOY_KEY/#\~/$HOME}"
if [ ! -f "$GITHUB_DEPLOY_KEY" ]; then
  echo "⚠️ GitHub deploy key not found at: $GITHUB_DEPLOY_KEY"
  echo "Set it with: GITHUB_DEPLOY_KEY=~/.ssh/id_ed25519_carla_deploy"
  exit 1
fi

# -----------------------------
# Extract SSH components
# -----------------------------
USER=$(echo "$SSH_CMD" | sed -n 's/ssh \([^@]*\)@.*/\1/p')
HOST=$(echo "$SSH_CMD" | sed -n 's/ssh [^@]*@\([^ ]*\).*/\1/p')
PORT=$(echo "$SSH_CMD" | sed -n 's/.*-p \([0-9]*\).*/\1/p')
KEY=$(echo "$SSH_CMD" | sed -n 's/.*-i \([^ ]*\).*/\1/p')

if [ -z "${PORT:-}" ]; then PORT=22; fi
if [ -z "${KEY:-}" ]; then KEY="$HOME/.ssh/id_ed25519"; fi
KEY="${KEY/#\~/$HOME}"

PUBKEY_PATH="${KEY}.pub"
if [ ! -f "$PUBKEY_PATH" ]; then
  echo "⚠️ Public key not found at: $PUBKEY_PATH"
  echo "Run: ssh-keygen -y -f \"$KEY\" > \"$PUBKEY_PATH\""
  exit 1
fi
PUBKEY="$(cat "$PUBKEY_PATH")"

# -----------------------------
# Update ~/.ssh/config (replace block)
# -----------------------------
echo "==> Writing SSH config entry (replace if exists)..."

mkdir -p "$HOME/.ssh"
touch "$CONFIG"

awk -v host="$ALIAS" '
  $1 == "Host" && $2 == host {skip=1; next}
  $1 == "Host" && skip==1 {skip=0}
  skip==0 {print}
' "$CONFIG" > "${CONFIG}.tmp"
mv "${CONFIG}.tmp" "$CONFIG"

cat >> "$CONFIG" <<EOF

Host $ALIAS
    HostName $HOST
    Port $PORT
    User $USER
    IdentityFile $KEY
    IdentitiesOnly yes
    RequestTTY no
EOF

echo "==> SSH config updated with host '$ALIAS'"

# -----------------------------
# Remote: ensure carla user + SSH key
# -----------------------------
echo "==> Ensuring 'carla' user and SSH key on remote..."

ssh -T "$ALIAS" 'bash -s' <<REMOTE
set -euo pipefail

if ! id -u carla >/dev/null 2>&1; then
  useradd -m -s /bin/bash carla
  echo "Created user 'carla'"
else
  echo "User 'carla' already exists"
fi

mkdir -p /home/carla/.ssh
chmod 700 /home/carla/.ssh

AUTH=/home/carla/.ssh/authorized_keys
touch "\$AUTH"
chmod 600 "\$AUTH"
grep -qxF "$PUBKEY" "\$AUTH" || echo "$PUBKEY" >> "\$AUTH"
chown -R carla:carla /home/carla/.ssh
REMOTE

echo "==> Installing GitHub deploy key for carla..."

# Copy deploy private key from Mac -> pod (do NOT commit this key)
scp -P "$PORT" -i "$KEY" "$GITHUB_DEPLOY_KEY" "$USER@$HOST:/home/carla/.ssh/id_ed25519_github"

# Fix perms + configure carla's ssh for GitHub
ssh -T "$ALIAS" "bash -s" <<'REMOTE'
set -euo pipefail

chmod 600 /home/carla/.ssh/id_ed25519_github
chown carla:carla /home/carla/.ssh/id_ed25519_github

su - carla -c '
cat > ~/.ssh/config <<EOF
Host github.com
  HostName github.com
  User git
  IdentityFile ~/.ssh/id_ed25519_github
  IdentitiesOnly yes
  StrictHostKeyChecking accept-new
EOF
chmod 600 ~/.ssh/config
ssh-keyscan -H github.com >> ~/.ssh/known_hosts 2>/dev/null || true
chmod 644 ~/.ssh/known_hosts
'
REMOTE

# -----------------------------
# Remote: update repo (as carla)
# -----------------------------
echo "==> Getting latest from GitHub (repo: $REPO_DIR)..."

ssh -T "$ALIAS" "bash -s" <<REMOTE
set -euo pipefail

# Run as carla
run_as_carla() { su - carla -c "\$*"; }
# GitHub SSH (key + known_hosts) is configured in the deploy-key step above; avoid duplicating here.

if [ -d "$REPO_DIR/.git" ]; then
  run_as_carla "cd '$REPO_DIR' && git fetch --prune && git pull --ff-only"
  echo "Repo updated: $REPO_DIR"
elif [ -n "$REPO_URL" ]; then
  mkdir -p "$(dirname "$REPO_DIR")"
  run_as_carla "git clone '$REPO_URL' '$REPO_DIR'"
  echo "Repo cloned: $REPO_DIR"
else
  echo "⚠️ Repo not found at $REPO_DIR and REPO_URL not set; skipping git update."
fi
REMOTE

# -----------------------------
# Remote: install project Python deps
# -----------------------------
echo "==> Installing project Python dependencies (requirements.txt)..."

ssh -T "$ALIAS" "bash -s" <<REMOTE
set -euo pipefail
if [ -f "$REPO_DIR/requirements.txt" ]; then
  python3 -m pip install -q -r "$REPO_DIR/requirements.txt"
  echo "Project dependencies installed."
else
  echo "No requirements.txt at $REPO_DIR; skipping."
fi
REMOTE

# -----------------------------
# Remote: start CARLA if not running (as carla)
# -----------------------------
echo "==> Starting CARLA if not already running (dir: $CARLA_DIR, port: $CARLA_PORT)..."

ssh -T "$ALIAS" "bash -s" <<REMOTE
set -euo pipefail

run_as_carla() { su - carla -c "\$*"; }

# Already running?
if pgrep -u carla -f "CarlaUE4.sh.*-carla-rpc-port=${CARLA_PORT}" >/dev/null 2>&1; then
  echo "CARLA already running on port ${CARLA_PORT}"
  exit 0
fi

# CARLA not installed?
if [ ! -x "$CARLA_DIR/CarlaUE4.sh" ]; then
  if [ "$INSTALL" = "1" ]; then
    if [ -f "$REPO_DIR/infra/install_carla.sh" ]; then
      echo "CARLA not found at $CARLA_DIR. Running install script..."
      if [ "\$(id -u)" -eq 0 ]; then
        START_CARLA=1 bash "$REPO_DIR/infra/install_carla.sh"
      else
        sudo START_CARLA=1 bash "$REPO_DIR/infra/install_carla.sh"
      fi
    else
      echo "Install script not found at $REPO_DIR/infra/install_carla.sh"
      echo "Ensure the repo is cloned (e.g. at $REPO_DIR) then run again without --no-install."
      exit 1
    fi
  else
    echo "CARLA not installed at $CARLA_DIR."
    echo "Next: install on the pod (as root), then start:"
    echo "  ssh $ALIAS 'bash $REPO_DIR/infra/install_carla.sh'"
    echo "Or run this script without --no-install to install and start automatically."
    exit 0
  fi
  exit 0
fi

# Start it
run_as_carla "nohup '$CARLA_DIR/CarlaUE4.sh' $CARLA_ARGS > /home/carla/carla_${CARLA_PORT}.log 2>&1 &"
sleep 1

if pgrep -u carla -f "CarlaUE4.sh.*-carla-rpc-port=${CARLA_PORT}" >/dev/null 2>&1; then
  echo "CARLA started (log: /home/carla/carla_${CARLA_PORT}.log)"
else
  echo "Tried to start CARLA but process not detected. Check log: /home/carla/carla_${CARLA_PORT}.log"
  exit 1
fi
REMOTE

echo "==> Done."
echo "Connect via:"
echo "  ssh $ALIAS"