"""Pace simulation against elapsed wall time, including sensor work."""


def next_tick(deadline, now, dt):
    # Catch up a single slow sensor frame. Bound debt after a long stall so the
    # loop returns to its normal cadence instead of racing indefinitely.
    deadline = max(deadline + dt, now - 0.2)
    return deadline, max(0.0, deadline - now)
