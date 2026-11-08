import enum


class Seat(enum.Enum):
    INSERTED_VERIFIED = 1
    NOT_INSERTED = 2
    UNKNOWN_SENSOR = 3
    FAULT = 4


def seat_state(samples, now, v):
    """samples: [(t, bool)] from the independent seating sensor. Verified only if every sample in the last
    stable_dwell_s equals success_value and the window is covered; stale or empty = UNKNOWN_SENSOR."""
    win = [(t, x) for t, x in samples if now - t <= v["stable_dwell_s"]]
    if not samples or now - samples[-1][0] > v["stale_s"] or not win or now - samples[0][0] < v["stable_dwell_s"]:
        return Seat.UNKNOWN_SENSOR
    return Seat.INSERTED_VERIFIED if all(x == v["success_value"] for _, x in win) else Seat.NOT_INSERTED
