#!/usr/bin/env bash

# Hard isolation
unset ROS_DISTRO
unset ROS_VERSION
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset GZ_SIM_RESOURCE_PATH
unset GZ_SIM_SYSTEM_PLUGIN_PATH
unset GAZEBO_MODEL_PATH
unset GAZEBO_PLUGIN_PATH

# Source Pixi ROS
source "install/setup.bash"

# Pixi env root (robust)
PIXI_ENV_ROOT="$(dirname "$(dirname "$(which gz)")")"

# ----------------------------
# Explicit library isolation
# ----------------------------
export LD_LIBRARY_PATH="$PIXI_ENV_ROOT/lib"
export CMAKE_PREFIX_PATH="$PIXI_ENV_ROOT"
export PKG_CONFIG_PATH="$PIXI_ENV_ROOT/lib/pkgconfig"
export PYTHONPATH="$PIXI_ENV_ROOT/lib/python3.*/site-packages"

# ----------------------------
# Gazebo Sim paths
# ----------------------------
export GZ_SIM_RESOURCE_PATH="$PIXI_ENV_ROOT/share"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PIXI_ENV_ROOT/lib"

# Optional: your robot models
export GZ_SIM_RESOURCE_PATH="$PIXI_ENV_ROOT/share/ergoCub/robots:$GZ_SIM_RESOURCE_PATH"