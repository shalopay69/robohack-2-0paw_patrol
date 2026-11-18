#!/bin/sh
set -e

case "${1:-}" in
    ping)
        printf '%s\n' '{"protocol_version":"1.0","supported_tasks":["T2"],"self_trained_rl":false,"preferred_speed_mps":null}'
        ;;
    T2)
        . /opt/ros/humble/setup.sh
        . /opt/unitree/local_setup.sh
        exec python3 /opt/go1/t2_node.py
        ;;
    *)
        printf 'Unsupported command: %s\n' "${1:-<missing>}" >&2
        exit 2
        ;;
esac
