# Insertion servo run: 2026-11-27T18:16:07

Outcome: **STOPPED** marker 10 lost: frames arrive but marker 10 is not in them (seen ids: [])

- Command: `ros2_iface/insert_servo.py --wire power --out /home/aryans-cat/Documents/GitHub/VBRM_insertion/insertion_task/Media/IBVS/run_20261127_181435_insert_power --speed 0.03 --teach-seated --command-file /tmp/claude-1000/-home-aryans-cat-Documents-GitHub-VBRM-insertion/a0e1d410-83d7-49d7-a49e-25ade124febc/scratchpad/seat_cmd`
- Wire: power, mode: continuous
- Code: {'commit': '2130bc6', 'uncommitted_files_in_IBVS': []}
- Not a verified insertion: the servo only brings the plug to the socket entrance; no seating sensor was used.
- Files: config.json (everything this run used), trajectory.csv (every observation), metrics.json, controller.log (console), screenshots/, videos/servo.mp4 (green = marker, red = goal), plots/.
