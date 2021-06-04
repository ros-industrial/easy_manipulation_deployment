#!/bin/bash
set -e

# setup moveit2 environment
source /opt/moveit2/install/setup.bash
exec "$@"
