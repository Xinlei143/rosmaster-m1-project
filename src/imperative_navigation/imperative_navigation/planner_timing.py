"""Keep the planner's discrete-time model aligned with real callback timing."""


class MeasuredPlannerPeriod:
    """Return a bounded elapsed planner period.

    The online planner uses its ``DT`` for track velocity, acceleration rollout
    and predicted positions.  A timer period alone is not sufficient when a
    planner callback takes longer than that timer period, as happens on the
    RDK X5.  This helper uses the actual elapsed monotonic time while retaining
    a conservative bound for startup pauses or debugger stops.
    """

    def __init__(self, configured_period, minimum_period=0.05, maximum_period=0.50):
        if configured_period <= 0.0:
            raise ValueError("configured_period must be positive")
        if minimum_period <= 0.0 or maximum_period < minimum_period:
            raise ValueError("invalid planner period bounds")
        self.configured_period = float(configured_period)
        self.minimum_period = float(minimum_period)
        self.maximum_period = float(maximum_period)
        self.last_time = None

    def next_period(self, now):
        now = float(now)
        if self.last_time is None:
            period = self.configured_period
        else:
            period = now - self.last_time
        self.last_time = now
        return min(self.maximum_period, max(self.minimum_period, period))
