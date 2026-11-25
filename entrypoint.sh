#!/bin/sh
set -e

case "${1:-}" in
    ping)
        printf '%s\n' '{"protocol_version":"1.0","supported_tasks":["T2","T3","T4"],"self_trained_rl":true,"preferred_speed_mps":0.4}'
        ;;
    T2)
        . /opt/ros/humble/setup.sh
        . /opt/unitree/local_setup.sh
        exec python3 /opt/go1/t2_node.py
        ;;
    T3)
        . /opt/ros/humble/setup.sh
        . /opt/unitree/local_setup.sh
        export OPENBLAS_NUM_THREADS=1
        exec python3 /opt/go1/t3_node.py
        ;;
    T4)
        . /opt/ros/humble/setup.sh
        export OPENBLAS_NUM_THREADS=1
        exec python3 /opt/go1/t4_node.py
        ;;
    *)
        printf 'Unsupported command: %s\n' "${1:-<missing>}" >&2
        exit 2
        ;;
esac
