#!/usr/bin/env bash
# minelab/scripts/health_check.sh
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Quick sanity check for the minelab node graph.
# Run after starting all nodes to verify that expected topics are active.
#
# Usage: bash minelab/scripts/health_check.sh

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

EXPECTED_TOPICS=(
    "/minelab/depth_features"
    "/minelab/marker_info"
    "/minelab/pilot_status"
    "/control/command"
    "/mavros/rc/override"
)

echo "=== minelab health check ==="
echo ""

ALL_OK=true

for topic in "${EXPECTED_TOPICS[@]}"; do
    # rostopic list exits 0 and prints topics; grep for ours
    if rostopic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "${GREEN}[OK]${NC}  ${topic}"
    else
        echo -e "${RED}[MISSING]${NC}  ${topic}"
        ALL_OK=false
    fi
done

echo ""
echo "--- Node list ---"
rosnode list 2>/dev/null | grep -E "depth_feature|marker_detection|pilot|vehicle_io|logger" \
    | while read -r n; do echo -e "${GREEN}[UP]${NC}  $n"; done || true

echo ""
if $ALL_OK; then
    echo -e "${GREEN}All expected topics are present.${NC}"
else
    echo -e "${YELLOW}Some topics are missing – check that all nodes are running.${NC}"
    exit 1
fi
