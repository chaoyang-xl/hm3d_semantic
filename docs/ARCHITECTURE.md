# Architecture

The package separates pure camera/depth/pose math from the lazy Habitat-Sim
adapter. `HabitatCapture` produces synchronized frames, `exporter` publishes a
transactional file dataset, and `validator` independently reopens that dataset.
No downstream reconstruction or robotics repository is imported.

