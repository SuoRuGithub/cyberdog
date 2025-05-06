ros2 launch camera_test stereo_camera.py >/dev/null 2>&1 &
ros2 lifecycle set /dog/camera/camera configure
ros2 lifecycle set /dog/camera/camera activate
ros2 lifecycle set /stereo_camera configure
ros2 lifecycle set /stereo_camera activate