# Insertion servo run: 2026-11-27T18:07:50

Outcome: **STOPPED** arm blocked: moved 1.7 mm where 4.0 mm was expected (wall or plug contact?): stopped

- Command: `ros2_iface/insert_servo.py --wire usb --out /home/aryans-cat/Documents/GitHub/VBRM_insertion/insertion_task/Media/IBVS/run_20261127_180616_insert_usb --speed 0.03 --teach-seated --command-file /tmp/claude-1000/-home-aryans-cat-Documents-GitHub-VBRM-insertion/a0e1d410-83d7-49d7-a49e-25ade124febc/scratchpad/seat_cmd`
- Wire: usb, mode: continuous
- Code: {'commit': '3f74367', 'uncommitted_files_in_IBVS': []}
- Not a verified insertion: the servo only brings the plug to the socket entrance; no seating sensor was used.
- Files: config.json (everything this run used), trajectory.csv (every observation), metrics.json, controller.log (console), screenshots/, videos/servo.mp4 (green = marker, red = goal), plots/.
