#!/bin/bash

use_jpl_mm=${1-false}

pkg_path=$(rospack find ros2_bridge)

python $pkg_path/scripts/generate_config.py \
  --config $pkg_path/config/master.yaml \
  --robots husky1 husky2 husky3 husky4 xmaxx1 spot1 spot2 spot3 spot4 \
  --output-directory $pkg_path/config/autogen \
  --use-jpl-mm $use_jpl_mm

python $pkg_path/scripts/generate_config.py \
  --config $pkg_path/config/master_drone.yaml \
  --robots scout1 \
  --output-directory $pkg_path/config/autogen_drone