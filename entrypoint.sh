#!/bin/sh
set -eu

case "${1:-}" in
    ping)
        printf '%s\n' '{"protocol_version":"1.0","supported_tasks":[],"self_trained_rl":false,"preferred_speed_mps":null}'
        ;;
    *)
        printf 'Unsupported command: %s\n' "${1:-<missing>}" >&2
        exit 2
        ;;
esac
