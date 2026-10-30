#!/usr/bin/env bash
# Sends one attitude setpoint to a running MonospinnerController, from any terminal.
#
# Usage: send_setpoint.sh <precession_deg> <nutation_deg> [model]
#   precession theta1: tilt direction, from world x (east) towards world y (north)
#   nutation   theta2: tilt magnitude from vertical (0 = upright)
# (paper Eq. 93; the controller takes radians, this script takes degrees)
#
# Publishes gz.msgs.Vector2d {x: theta1, y: theta2} on /<model>/controller/setpoint, the
# topic MonospinnerController subscribes to (src/MonospinnerController.cc, OnSetpoint). The
# controller logs every setpoint it receives, and records it as n_bar in controller.csv.
#
# Typical use: start a run that plays no setpoints of its own,
#   SETPOINTS="" STATE_SOURCE=ground_truth RECORD_S=120 run_closed_loop.sh
# then, from a second terminal, once the vehicle hovers:
#   send_setpoint.sh 90 45     # tilt 45 deg towards north
#   send_setpoint.sh 0 0       # back upright
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "usage: $(basename "$0") <precession_deg> <nutation_deg> [model]" >&2
  exit 2
fi
MODEL="${3:-monospinner}"

read -r theta1 theta2 < <(python3 -c "import math, sys; \
print(math.radians(float(sys.argv[1])), math.radians(float(sys.argv[2])))" "$1" "$2") || {
  echo "error: precession and nutation must be numbers (degrees)." >&2
  exit 2
}

gz topic -t "/$MODEL/controller/setpoint" -m gz.msgs.Vector2d -p "x: $theta1, y: $theta2"
echo "sent setpoint: precession $1 deg, nutation $2 deg -> /$MODEL/controller/setpoint"
