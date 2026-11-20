#!/usr/bin/env bash
# Stops the stack started by run_stack_method2.sh (driver, camera, MoveIt, grasp node, RViz scene, fixture publisher). Safe to run if nothing is running.
PG=$(ps -eo pid=,pgid=,args= | grep -E "bash .*run_stack_method2\.sh" | grep -v grep | awk '{print $2}' | head -1)
if [ -n "$PG" ]; then kill -INT -- -"$PG"; sleep 6; echo "stack stopped"; else echo "no stack running"; fi
