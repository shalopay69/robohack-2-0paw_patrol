#!/bin/bash
set -euo pipefail

tests_dir=$(cd "$(dirname "$0")" && pwd)
sim_repo=${1:?Usage: bash tests/check_ros_stream.sh /path/to/go1-simulator-public}
image=${2:-go1-participant:t2-local}
cd "$sim_repo"
export ROS_DOMAIN_ID=${T2_TEST_ROS_DOMAIN_ID:-167}
sim_log=$(mktemp)
python3 -u run_local.py --task T2 --headless --timeout 20 --domain "$ROS_DOMAIN_ID" \
    --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
    --participant-image "$image" >"$sim_log" 2>&1 &
launcher=$!

cleanup() {
    kill -TERM "$launcher" 2>/dev/null || true
    wait "$launcher" || true
    cat "$sim_log"
    rm -f -- "$sim_log"
}
trap cleanup EXIT

for _ in {1..120}; do
    if grep -q 'Pair started' "$sim_log"; then
        break
    fi
    kill -0 "$launcher" || exit 1
    sleep 0.25
done
grep -q 'Pair started' "$sim_log"

export CYCLONEDDS_URI=$(cat user_data/ros2_custom_config_setup/cyclonedds_local.xml)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
timeout 35s docker run --rm --pull=never --network host \
    --env ROS_DOMAIN_ID --env CYCLONEDDS_URI --env RMW_IMPLEMENTATION --env ROS_LOCALHOST_ONLY \
    --volume "$tests_dir/check_ros_stream.py:/check_ros_stream.py:ro" \
    --entrypoint /bin/bash "$image" -ec \
    'source /opt/ros/humble/setup.bash; source /opt/unitree/local_setup.bash; exec python3 /check_ros_stream.py'
