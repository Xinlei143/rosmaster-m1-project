"""Fast receding-horizon velocity rollout for the holonomic Rosmaster M1.

Unlike a waypoint bypass, every control cycle samples continuous velocity
directions and scores their whole predicted paths against the latest laser
cloud and predicted moving tracks.  Repeatedly selecting nearby velocities
creates a smooth curve while preserving a direct route in open space.
"""

import math

import torch


def goal_is_dynamically_blocked(goal, tracks, robot_clearance, dynamic_radius):
    """Whether a confirmed moving object occupies the arrival region now."""
    moving = [track for track in tracks if torch.linalg.norm(track["velocity"]) > 0.0]
    if not moving:
        return False
    centres = torch.stack([track["position"] for track in moving]).to(goal)
    blocked_distance = float(robot_clearance) + float(dynamic_radius) + 0.08
    return bool(torch.any(torch.linalg.norm(centres - goal[None], dim=1) <= blocked_distance))


def choose_velocity(position, velocity, goal, obstacle_points, tracks, *, max_speed,
                    max_acceleration, robot_clearance, horizon=16, dt=0.10,
                    heading_samples=41, speed_samples=4, dynamic_radius=0.20,
                    reference_velocity=None, yielding=False):
    """Return ``(command_velocity, predicted_path, minimum_clearance)``.

    Raw LiDAR returns are treated as static over the short horizon.  Track
    centres are separately propagated with their estimated velocities, which
    lets the same rollout yield to a moving person/robot without a special
    state machine.
    """
    device, dtype = position.device, position.dtype
    goal_offset = goal - position
    goal_distance = torch.linalg.norm(goal_offset).clamp_min(1e-6)
    goal_angle = torch.atan2(goal_offset[1], goal_offset[0])
    headings = goal_angle + torch.linspace(-math.pi, math.pi, heading_samples,
                                            device=device, dtype=dtype)
    speeds = torch.linspace(0.0, float(max_speed), speed_samples + 1,
                            device=device, dtype=dtype)[1:]
    target_velocities = (speeds[:, None, None] * torch.stack(
        (torch.cos(headings), torch.sin(headings)), dim=1)[None]).reshape(-1, 2)
    # Stopping remains a valid choice whenever no collision-free trajectory
    # exists, rather than leaving an earlier command active.
    target_velocities = torch.cat((torch.zeros(1, 2, device=device, dtype=dtype),
                                   target_velocities), dim=0)
    count = len(target_velocities)
    predicted_positions = torch.empty(count, horizon, 2, device=device, dtype=dtype)
    predicted_velocities = torch.empty_like(predicted_positions)
    rolling_position = position.repeat(count, 1)
    rolling_velocity = velocity.repeat(count, 1)
    max_delta = float(max_acceleration) * float(dt)
    for step in range(horizon):
        delta = target_velocities - rolling_velocity
        delta *= torch.clamp(max_delta / torch.linalg.norm(delta, dim=1).clamp_min(1e-6), max=1.0)[:, None]
        rolling_velocity += delta
        rolling_position += float(dt) * rolling_velocity
        predicted_positions[:, step] = rolling_position
        predicted_velocities[:, step] = rolling_velocity

    obstacle_cost = torch.zeros(count, device=device, dtype=dtype)
    min_clearance = torch.full((count,), float("inf"), device=device, dtype=dtype)
    if len(obstacle_points):
        distances = torch.linalg.norm(
            obstacle_points[None, None] - predicted_positions[:, :, None], dim=3)
        min_clearance = torch.minimum(min_clearance, distances.amin(dim=(1, 2)))
        # A finite soft field guides the robot early into the wider side;
        # the hard term rejects trajectories whose footprint intersects a
        # laser return.
        obstacle_cost += 70.0 * torch.sum(torch.relu(robot_clearance + 0.10 - distances) ** 2,
                                          dim=(1, 2))

    if tracks:
        centres = torch.stack([track["position"] for track in tracks]).to(device=device, dtype=dtype)
        velocities = torch.stack([track["velocity"] for track in tracks]).to(device=device, dtype=dtype)
        times = torch.arange(1, horizon + 1, device=device, dtype=dtype) * float(dt)
        future_centres = centres[None] + times[:, None, None] * velocities[None]
        distances = torch.linalg.norm(future_centres[None] - predicted_positions[:, :, None], dim=3)
        dynamic_clearance = float(robot_clearance) + float(dynamic_radius)
        min_clearance = torch.minimum(min_clearance, distances.amin(dim=(1, 2)) - float(dynamic_radius))
        obstacle_cost += 100.0 * torch.sum(torch.relu(dynamic_clearance + 0.12 - distances) ** 2,
                                           dim=(1, 2))

    collision = min_clearance < float(robot_clearance)
    endpoint_cost = 7.0 * torch.sum((predicted_positions[:, -1] - goal) ** 2, dim=1)
    reference_velocity = velocity if reference_velocity is None else reference_velocity
    # This is the trajectory hysteresis term: nearby candidate velocities
    # cost less than a left/right reversal, so sensor noise cannot make the
    # M1 alternate between escape directions on adjacent frames.
    velocity_change_cost = 4.0 * torch.sum((target_velocities - reference_velocity[None]) ** 2, dim=1)
    # Prefer forward progress when two paths have equivalent clearance, but
    # do not force forward motion if the safe curve initially needs side-slip.
    progress_weight = 0.15 if yielding else 1.5
    progress_cost = -progress_weight * torch.sum((predicted_positions[:, -1] - position[None]) *
                                     (goal_offset / goal_distance)[None], dim=1)
    total_cost = endpoint_cost + velocity_change_cost + progress_cost + obstacle_cost
    total_cost = total_cost + collision.to(dtype) * 1_000_000.0
    winner = torch.argmin(total_cost)
    return predicted_velocities[winner, 0], predicted_positions[winner], float(min_clearance[winner])
