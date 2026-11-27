# Insertion servo run: 2026-11-27T16:52:51

Outcome: **STOPPED** marker 10 lost: frames arrive but marker 10 is not in them (seen ids: [])

- Command: `Code/IBVS/ros2_iface/insert_servo.py --wire power --speed 0.03 --back-off-m 0`
- Wire: power, mode: continuous
- Code: {'commit': '570761f', 'uncommitted_files_in_IBVS': []}
- Not a verified insertion: the servo only brings the plug to the socket entrance; no seating sensor was used.
- Files: config.json (everything this run used), trajectory.csv (every observation), metrics.json, controller.log (console), screenshots/, videos/servo.mp4 (green = marker, red = goal), plots/.
