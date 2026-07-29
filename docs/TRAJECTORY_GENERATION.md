# Trajectory Generation

Waypoint mode samples a navigable goal and a Habitat shortest path, subdivides
the path to the configured forward step, rotates in place while heading error
exceeds the alignment tolerance, and only then moves via `try_step()`. Turns
therefore consume frames and reduce distance for a fixed frame budget. Replay
mode rejects invalid quaternions and off-navmesh positions.

