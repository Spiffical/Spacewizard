#!/usr/bin/env bash
# Copy this checkout to the Raspberry Pi and restart the rig there.
# Backs up the previous copy, keeps the Pi's Python environment, runtime state and
# sound files, runs the tests on the Pi, and (re)installs the systemd service.
#
#   scripts/deploy_to_pi.sh                                  # sbialek@spacewizard-pi.local, ~/Spacewizard-sound
#   scripts/deploy_to_pi.sh pi@raspberrypi.local spacewizard # remote dir is relative to the Pi's home

set -euo pipefail

HOST=${1:-${SPACEWIZARD_PI:-sbialek@spacewizard-pi.local}}
REMOTE_DIR=${2:-${SPACEWIZARD_PI_DIR:-Spacewizard-sound}}
REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SERVICE=spacewizard-production.service

printf 'Deploying %s to %s:~/%s\n' "$REPO_ROOT" "$HOST" "$REMOTE_DIR"
ssh -n "$HOST" "systemctl --user stop $SERVICE 2>/dev/null || true
    mkdir -p ~/spacewizard-backups
    if [ -d ~/$REMOTE_DIR ]; then
        tar czf ~/spacewizard-backups/\$(date +%Y%m%d_%H%M%S)-before-deploy.tgz --exclude=.venv -C ~ $REMOTE_DIR
    fi"

# Excluded paths are never deleted on the Pi; sound files there are protected too.
rsync -az --delete \
    --exclude '/.git/' --exclude '/.venv/' --exclude '/.runtime/' --exclude '/local/' \
    --exclude '/data/' --exclude '/docs/' --exclude '/legacy/' \
    --exclude '__pycache__/' --exclude '.DS_Store' \
    --filter 'P /synth/samples/***' \
    "$REPO_ROOT/" "$HOST:$REMOTE_DIR/"

ssh -n "$HOST" "set -e; cd ~/$REMOTE_DIR
    [ -x .venv/bin/python ] || python3 -m venv .venv
    .venv/bin/python -m pip install -q -r requirements.txt
    .venv/bin/python -m unittest -q tests/test_reliability.py
    scripts/install_pi_service.sh > /dev/null
    systemctl --user restart $SERVICE
    echo 'Deployed and restarted.'"
