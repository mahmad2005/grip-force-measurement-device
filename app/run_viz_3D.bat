@echo off
REM Launch the 32x64 grip pressure + IMU visualizer with 3D cube
python "%~dp0viz_full_with_gyro_accel_mag_3Dobj.py" --show-3d --show-magnitude %*
