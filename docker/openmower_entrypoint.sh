#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/open_mower_ros/devel/setup.bash

# setup om environment - source version env, then re-export from the version
# string file as a robust fallback (some shell-launch chains drop env vars from
# `source`d files; reading the plain string into an explicit export survives).
source /opt/open_mower_ros/version_info.env
export OM_SOFTWARE_VERSION="${OM_SOFTWARE_VERSION:-$(cat /opt/open_mower_ros/.version_string 2>/dev/null || echo unknown)}"

# OSv2 debugging get controlled via env var DEBUG and has the ROSCONSOLE_CONFIG_FILE embedded
shopt -s nocasematch
case "${DEBUG:-0}" in
    1|true|yes|on|y)
        export ROSOUT_DISABLE_FILE_LOGGING=False
        unset ROSCONSOLE_CONFIG_FILE
    ;;
    *)
        export ROSCONSOLE_CONFIG_FILE=/config/rosconsole.config
        export ROSOUT_DISABLE_FILE_LOGGING=True
    ;;
esac
shopt -u nocasematch || true

# Ensure stdout and stderr are unbuffered to get logging in real time order
export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
export PYTHONUNBUFFERED=1

exec -- "$@"
