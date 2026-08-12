"""Adapter-side temporal filtering for noisy laser-derived obstacle tracks.

The core Imperative planner deliberately remains unchanged.  This module is
used only by the ROS adapters: raw laser points still enter every collision
cost, while transient circle fits are kept out of long-horizon moving-obstacle
prediction until they have been observed consistently.
"""

import torch


class ConfirmedTrackFilter:
    """Expose smoothed, repeatedly observed tracks without mutating raw tracks."""

    def __init__(self, confirmation_age=3, position_alpha=0.35,
                 static_speed_threshold=0.25):
        if confirmation_age < 1:
            raise ValueError("confirmation_age must be at least one")
        if not 0.0 < position_alpha <= 1.0:
            raise ValueError("position_alpha must be in (0, 1]")
        if static_speed_threshold < 0.0:
            raise ValueError("static_speed_threshold must be non-negative")
        self.confirmation_age = int(confirmation_age)
        self.position_alpha = float(position_alpha)
        self.static_speed_threshold = float(static_speed_threshold)
        self._positions = {}

    def update(self, raw_tracks):
        """Return fresh confirmed track copies with EMA-smoothed centers.

        New or missed tracks remain available as raw scan points to the
        planner, but are not predicted as moving circular obstacles.  A
        low-speed confirmed track is treated as static for prediction; its
        laser returns still remain collision obstacles at every control cycle.
        """
        active_ids = set()
        confirmed_tracks = []
        for raw_track in raw_tracks:
            track_id = raw_track["id"]
            active_ids.add(track_id)
            raw_position = raw_track["position"]
            previous = self._positions.get(track_id)
            if previous is None:
                position = raw_position.clone()
            else:
                position = (self.position_alpha * raw_position +
                            (1.0 - self.position_alpha) * previous)
            self._positions[track_id] = position.clone()

            if (raw_track["age"] < self.confirmation_age or
                    raw_track["missed"] != 0):
                continue

            velocity = raw_track["velocity"].clone()
            if torch.linalg.norm(velocity) < self.static_speed_threshold:
                velocity.zero_()
            track = dict(raw_track)
            track["position"] = position
            track["velocity"] = velocity
            confirmed_tracks.append(track)

        for track_id in set(self._positions) - active_ids:
            del self._positions[track_id]
        return confirmed_tracks
