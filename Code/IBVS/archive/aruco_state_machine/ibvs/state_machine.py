"""NOT_READY -> CALIBRATE_CHECK -> PICK -> VERIFY_GRASP -> PREVIEW -> SERVO_ALIGN -> GUARD_CHECK -> ATTEMPT -> VERIFY
-> DONE | RETRACT -> SEARCH_NEXT | ABORT.   Modes: B0 taught pose (no servo), A1 servo + one attempt, A2 servo + search.

`p` supplies the motion/sensing primitives (ROS node in real runs, fakes in tests). Each returns ok/status; anything
but success aborts. No transition continues into contact after a fault or an unknown sensor.
"""
from .verify import Seat


def run_cycle(p, mode, cands, log, max_total_time_s, clock):
    t0 = clock()
    tries = {"n": 0}

    def go(state, reason="ok"):
        log.row(phase=state, stop_reason=reason, attempt_index=tries["n"], controller=mode)

    def abort(reason):
        p.abort(reason)
        go("ABORT", reason)
        return "ABORT", reason

    go("NOT_READY")
    for state, fn in (("CALIBRATE_CHECK", p.check_calibration), ("PICK", p.pick), ("VERIFY_GRASP", p.verify_grasp),
                      ("PREVIEW", p.preview)):
        go(state)
        r = fn()
        if r is not True:
            return abort(f"{state}:{r}")
    todo = list(cands) if mode == "A2" else [(0.0, 0.0, 0.0)]
    while todo:
        cand = todo.pop(0)
        if mode != "B0":
            go("SERVO_ALIGN")
            r = p.servo_align()
            if r is not True:
                return abort(f"SERVO_ALIGN:{r}")
        if cand != (0.0, 0.0, 0.0):
            r = p.apply_search_offset(cand)
            if r is not True:
                return abort(f"SEARCH_OFFSET:{r}")
        go("GUARD_CHECK")
        r = p.guard_check()
        if r is not True:
            return abort(f"GUARD_CHECK:{r}")
        go("ATTEMPT")
        r = p.attempt()
        if r not in ("OK", "NO_PROGRESS"):       # NO_PROGRESS = clean failed attempt; anything else = fault
            return abort(f"ATTEMPT:{r}")
        go("VERIFY")
        seat = p.verify_seated()
        if seat is Seat.INSERTED_VERIFIED:
            log.row(phase="DONE", verified_success=True, attempt_index=tries["n"], controller=mode)
            return "DONE", "verified"
        if seat is not Seat.NOT_INSERTED:         # UNKNOWN_SENSOR / FAULT: never keep pushing
            return abort(f"VERIFY:{seat.name}")
        tries["n"] += 1
        go("RETRACT")
        r = p.retract()
        if r is not True:
            return abort(f"RETRACT:{r}")
        if not todo:
            break
        if clock() - t0 > max_total_time_s:
            return abort("SEARCH_TIMEOUT")
        go("SEARCH_NEXT")
    log.row(phase="DONE", verified_success=False, attempt_index=tries["n"], controller=mode)
    return "DONE", "not_inserted"
