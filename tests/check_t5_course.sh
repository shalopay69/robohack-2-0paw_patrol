#!/bin/bash
set -euo pipefail
tests_dir=$(cd "$(dirname "$0")" && pwd)
sim_repo=${1:?Pass the path to go1-simulator-public}
image=${2:-go1-participant:t5-local}
width=${3:-1.0}
shift "$(( $# < 3 ? $# : 3 ))"
cd "$sim_repo"
export ROS_DOMAIN_ID=${T5_TEST_ROS_DOMAIN_ID:-166}
export CYCLONEDDS_URI=$(cat user_data/ros2_custom_config_setup/cyclonedds_local.xml)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
sim_log=$(mktemp)
python3 -u run_local.py --task T5 --headless --timeout 130 --course-width "$width" --domain "$ROS_DOMAIN_ID" \
    --sim-image registry.cookcloud.ru/go1-hackathon-remote-stage/go1-sim:v1.0 \
    --participant-image "$image" >"$sim_log" 2>&1 &
launcher=$!

cleanup() {
    participant=$(sed -n 's/^Participant=//p' "$sim_log" | head -1)
    if [[ -n "$participant" ]]; then docker logs --tail 80 "$participant" >&2 2>&1 || true; fi
    kill -TERM "$launcher" 2>/dev/null || true
    wait "$launcher" || true
    cat "$sim_log"
    rm -f -- "$sim_log"
}
trap cleanup EXIT
for _ in {1..570}; do
    if grep -q 'Pair started' "$sim_log"; then break; fi
    kill -0 "$launcher" || exit 1
    sleep 0.25
done
grep -q 'Pair started' "$sim_log"
cat "$sim_log"

docker run --rm --pull=never --network host \
    --env ROS_DOMAIN_ID --env CYCLONEDDS_URI --env RMW_IMPLEMENTATION --env ROS_LOCALHOST_ONLY \
    --volume "$tests_dir:/audit:ro" --entrypoint /bin/bash "$image" -ec \
    'source /opt/ros/humble/setup.bash; exec python3 /audit/check_t5_course.py "$@"' bash "$@"
