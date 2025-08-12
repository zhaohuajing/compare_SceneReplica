#!/bin/bash

set -x
set -e

export PYTHONUNBUFFERED="True"

# 25/08/11: Prefer PyQt5’s plugin dirs
export QT_PLUGIN_PATH=/usr/local/lib/python3.8/dist-packages/PyQt5/Qt/plugins
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/local/lib/python3.8/dist-packages/PyQt5/Qt/plugins/platforms
export QT_QPA_PLATFORM=xcb
export ETS_TOOLKIT=qt
export QT_API=pyqt5
unset QT_DEBUG_PLUGINS   # optional; keep it if you want verbose output

python ./ros/test_model_with_segmentation.py --refine_steps 25 --num_grasp_samples 80 --batch_size 80 --threshold 0.9 --choose_fn better_than_threshold_in_sequence
