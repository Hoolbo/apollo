#!/bin/bash

# Test script for mycar_pos debugging

echo "=== Stopping mycar_pos ==="
docker exec apollo_neo_dev_hoolbo bash -c "cyber_launch stop /apollo_workspace/modules/drivers/mycar_pos/launch/mycar_pos.launch"

sleep 2

echo "=== Starting mycar_pos ==="
docker exec apollo_neo_dev_hoolbo bash -c "cyber_launch start /apollo_workspace/modules/drivers/mycar_pos/launch/mycar_pos.launch"

sleep 3

echo "=== Checking latest log ==="
LOG_FILE=$(docker exec apollo_neo_dev_hoolbo bash -c "ls -t /apollo/data/log/mycar_pos.log.INFO.* | head -1")
echo "Log file: $LOG_FILE"

echo ""
echo "=== Last 50 lines of log ==="
docker exec apollo_neo_dev_hoolbo tail -50 "$LOG_FILE"

echo ""
echo "=== Monitoring /apollo/localization/pose for 5 seconds ==="
timeout 5 docker exec apollo_neo_dev_hoolbo bash -c "cyber_monitor -c /apollo/localization/pose" || true
