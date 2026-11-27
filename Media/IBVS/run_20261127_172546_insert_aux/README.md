# Insertion servo run: 2026-11-27T17:26:36

Outcome: **STOPPED** marker 11 lost: frames arrive but marker 11 is not in them (seen ids: [10])

- Command: `ros2_iface/insert_servo.py --wire aux --out /home/aryans-cat/Documents/GitHub/VBRM_insertion/insertion_task/Media/IBVS/run_20261127_172546_insert_aux --speed 0.03`
- Wire: aux, mode: continuous
- Code: {'commit': 'e10165c', 'uncommitted_files_in_IBVS': []}
- Not a verified insertion: the servo only brings the plug to the socket entrance; no seating sensor was used.
- Files: config.json (everything this run used), trajectory.csv (every observation), metrics.json, controller.log (console), screenshots/, videos/servo.mp4 (green = marker, red = goal), plots/.
