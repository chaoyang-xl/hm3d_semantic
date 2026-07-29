# Trajectory Generation

Waypoint mode samples a navigable goal and a Habitat shortest path, subdivides
the path to the configured forward step, rotates in place while heading error
exceeds the alignment tolerance, and only then moves via `try_step()`. Turns
therefore consume frames and reduce distance for a fixed frame budget. Replay
mode rejects invalid quaternions and off-navmesh positions.


## Interactive Mode

Interactive capture opens a live RGB window and starts paused. `W` and `S`
move forward and backward through `PathFinder.try_step`; `A` and `D` rotate in
place. `R` starts recording, `P` pauses recording, and movement remains enabled
while paused. `Q` validates and publishes all recorded frames. After recording starts, `Esc`
or closing the window validates the recorded frames and retains a complete
`.partial` dataset. Movement before the first `R` press is preview-only. `--frames` is
the maximum number of recorded frames and reaching it saves automatically.
