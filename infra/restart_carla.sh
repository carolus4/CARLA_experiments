#!/usr/bin/env bash
# Restart the CARLA simulator (stop existing process, then start again).
# Run on the machine where CARLA is installed (e.g. inside the pod/container).
#
# Usage:
#   ./restart_carla.sh           # stop, then start
#   ./restart_carla.sh --stop    # stop only
#   ./restart_carla.sh --start   # start only (no-op if already running)
#
# Env (match install_carla.sh / setup_runpod_carla.sh):
#   CARLA_ROOT   default: /workspace/carla if /workspace writable, else /opt/carla
#   CARLA_PORT   default 2000
#   CARLA_USER   default carla (used when run as root to start CARLA)
set -euo pipefail

if [ -d "/workspace" ] && [ -w "/workspace" ] && [ -d "/workspace/carla" ]; then
  DEFAULT_CARLA_ROOT="/workspace/carla"
else
  DEFAULT_CARLA_ROOT="/opt/carla"
fi
CARLA_ROOT="${CARLA_ROOT:-$DEFAULT_CARLA_ROOT}"
CARLA_PORT="${CARLA_PORT:-2000}"
CARLA_USER="${CARLA_USER:-carla}"
STOP_ONLY=0
START_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --stop)  STOP_ONLY=1 ;;
    --start) START_ONLY=1 ;;
    -h|--help)
      echo "Usage: $0 [--stop|--start]"
      echo "  --stop   Stop CARLA only."
      echo "  --start  Start CARLA only (skip stop)."
      echo "  (none)   Restart: stop then start."
      exit 0
      ;;
  esac
done

do_stop() {
  echo "Stopping CARLA (port ${CARLA_PORT})..."
  if pkill -f "CarlaUE4" 2>/dev/null; then
    echo "  Sent SIGTERM to CARLA process(es). Waiting for exit..."
    for i in 1 2 3 4 5 6 7 8 9 10; do
      sleep 1
      if ! pgrep -f "CarlaUE4" >/dev/null 2>&1; then
        echo "  CARLA stopped."
        return 0
      fi
    done
    echo "  Still running after 10s. Sending SIGKILL..."
    pkill -9 -f "CarlaUE4" 2>/dev/null || true
    sleep 1
  fi
  if pgrep -f "CarlaUE4" >/dev/null 2>&1; then
    echo "  Warning: CARLA process may still be running. Check with: pgrep -af CarlaUE4"
    return 1
  fi
  echo "  CARLA stopped."
  return 0
}

do_start() {
  echo "Starting CARLA at ${CARLA_ROOT} (port ${CARLA_PORT})..."
  if pgrep -f "CarlaUE4" >/dev/null 2>&1; then
    echo "  CARLA is already running. Nothing to do."
    return 0
  fi
  if [ ! -x "${CARLA_ROOT}/CarlaUE4.sh" ]; then
    echo "  Error: CarlaUE4.sh not found or not executable at ${CARLA_ROOT}/CarlaUE4.sh"
    echo "  Install CARLA first, e.g.: sudo bash infra/install_carla.sh"
    return 1
  fi
  # Prefer the helper script if present (sets Xvfb, log, args)
  if [ -x "${CARLA_ROOT}/bin/start_carla.sh" ]; then
    if [ "$(id -u)" -eq 0 ] && id -u "$CARLA_USER" >/dev/null 2>&1; then
      su - "$CARLA_USER" -c "export CARLA_ROOT=${CARLA_ROOT} CARLA_PORT=${CARLA_PORT}; ${CARLA_ROOT}/bin/start_carla.sh"
    else
      export CARLA_ROOT CARLA_PORT
      bash "${CARLA_ROOT}/bin/start_carla.sh"
    fi
  else
    export DISPLAY="${DISPLAY:-:99}"
    CARLA_ARGS="${CARLA_ARGS:--RenderOffScreen -nosound -opengl -carla-rpc-port=${CARLA_PORT}}"
    LOG="${LOG:-/tmp/carla_${CARLA_PORT}.log}"
    # CARLA refuses to run as root; start as CARLA_USER when possible
    if [ "$(id -u)" -eq 0 ] && id -u "$CARLA_USER" >/dev/null 2>&1; then
      # So carla user can write the log (e.g. if it was left root-owned from a previous run)
      [ -f "$LOG" ] && chown "$CARLA_USER" "$LOG" 2>/dev/null || true
      su - "$CARLA_USER" -c "export DISPLAY=${DISPLAY:-:99}; cd ${CARLA_ROOT} && nohup ./CarlaUE4.sh ${CARLA_ARGS} > ${LOG} 2>&1 &"
    else
      cd "$CARLA_ROOT"
      nohup ./CarlaUE4.sh $CARLA_ARGS > "$LOG" 2>&1 &
    fi
    echo "  Waiting for CARLA process (can take 30–60s)..."
    for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20; do
      sleep 3
      if pgrep -f "CarlaUE4" >/dev/null 2>&1; then
        break
      fi
    done
    if ! pgrep -f "CarlaUE4" >/dev/null 2>&1; then
      echo "  CARLA process did not appear after 60s. Check: $LOG"
      [ -f "$LOG" ] && { echo "  Last 15 lines of log:"; tail -15 "$LOG" | sed 's/^/    /'; }
      return 1
    fi
    echo "  Process running. Waiting for RPC port ${CARLA_PORT} (can take 30–90s more)..."
    for _ in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24; do
      sleep 5
      if (command -v nc >/dev/null 2>&1 && nc -z 127.0.0.1 "$CARLA_PORT" 2>/dev/null) || \
         (timeout 2 bash -c "echo >/dev/tcp/127.0.0.1/$CARLA_PORT" 2>/dev/null); then
        echo "  CARLA started and ready. Log: $LOG"
        return 0
      fi
    done
    echo "  RPC port ${CARLA_PORT} not ready after 120s. Check: $LOG"
    [ -f "$LOG" ] && { echo "  Last 15 lines of log:"; tail -15 "$LOG" | sed 's/^/    /'; }
    return 1
  fi
  return 0
}

if [ "$START_ONLY" -eq 1 ]; then
  do_start
  exit $?
fi

do_stop
# Brief pause so port and resources are released
sleep 2

if [ "$STOP_ONLY" -eq 1 ]; then
  echo "Stop only. To start CARLA run: $0 --start"
  exit 0
fi

do_start
