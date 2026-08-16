"""Adapter-side confirmation and motion classification for Kalman tracks."""

import torch


class ConfirmedTrackFilter:
    """Expose repeatedly observed tracks without smoothing KF positions twice."""

    def __init__(self, confirmation_age=3, static_speed_threshold=0.25,
                 moving_confirmation_age=1):
        if confirmation_age < 1:
            raise ValueError("confirmation_age must be at least one")
        if static_speed_threshold < 0.0:
            raise ValueError("static_speed_threshold must be non-negative")
        if moving_confirmation_age < 1:
            raise ValueError("moving_confirmation_age must be at least one")
        self.confirmation_age = int(confirmation_age)
        self.static_speed_threshold = float(static_speed_threshold)
        self.moving_confirmation_age = int(moving_confirmation_age)
        self._moving_observations = {}

    def update(self, raw_tracks):
        """Return fresh confirmed track copies with classified velocities.

        New or missed tracks remain available as raw scan points to the
        planner, but are not exposed as tracked obstacles. A
        low-speed confirmed track is treated as static for prediction; its
        laser returns still remain collision obstacles at every control cycle.
        """
        active_ids = set()
        confirmed_tracks = []
        for raw_track in raw_tracks:
            track_id = raw_track["id"]
            active_ids.add(track_id)
            if (raw_track["age"] < self.confirmation_age or
                    raw_track["missed"] != 0):
                continue

            velocity = raw_track["velocity"].clone()
            moving_now = torch.linalg.norm(velocity) >= self.static_speed_threshold
            self._moving_observations[track_id] = (
                self._moving_observations.get(track_id, 0) + 1 if moving_now else 0)
            # Predict motion only after consecutive fresh observations support
            # it; raw laser returns protect every trajectory regardless.
            if self._moving_observations[track_id] < self.moving_confirmation_age:
                velocity.zero_()
            track = dict(raw_track)
            track["velocity"] = velocity
            confirmed_tracks.append(track)

        for track_id in set(self._moving_observations) - active_ids:
            self._moving_observations.pop(track_id, None)
        return confirmed_tracks
