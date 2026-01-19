#!/bin/bash
# colcon build wrapper - ビルドログを log/build に出力
cd /home/samatsum/f1tenth_ws
colcon --log-base log/build build "$@"
