# Insertion servo run: 2026-11-27T17:11:37

Outcome: **STOPPED** arm blocked: moved 1.7 mm where 4.4 mm was expected (wall or plug contact?): stopped

- Command: `ros2_iface/insert_servo.py --wire usb --out /home/aryans-cat/Documents/GitHub/VBRM_insertion/insertion_task/Media/IBVS/run_20261127_171004_insert_usb`
- Wire: usb, mode: continuous
- Code: {'commit': '4d4b7ff', 'uncommitted_files_in_IBVS': []}
- Not a verified insertion: the servo only brings the plug to the socket entrance; no seating sensor was used.
- Files: config.json (everything this run used), trajectory.csv (every observation), metrics.json, controller.log (console), screenshots/, videos/servo.mp4 (green = marker, red = goal), plots/.
