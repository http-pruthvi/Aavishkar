@echo off
call colcon build --symlink-install
call install\setup.bat
ros2 launch v2v_prototype v2v.launch.py
