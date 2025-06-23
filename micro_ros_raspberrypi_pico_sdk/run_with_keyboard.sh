sudo -E env \                                                                                                                                       1 ↵ dev@raspberrypi
"PYTHONPATH=$(python3 -c 'import sys; print(":".join(sys.path))')" \
"LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/local/lib:$LD_LIBRARY_PATH" \
python3 keyboard_publisher.py
