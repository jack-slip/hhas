# hhas [WIP]
Efficient Hybrid A-Star Implementation for Holonomic/Non-Holonomic Constraints
## Implementation Details
- Efficient Core A-star algorithm
- CRTP Pattern for injecting problem and state definitions
- Nanoflann kd-tree for fast nearest obstacle lookup in feasibility calculations
- OMP parralelization in feasibility calculations (sub 10ms for 500x500 grid with 72 bin size for SE2 orientation)
## TODO
- Implement Holonomic SE2 Grid A-star with Rectangular Footprint
- Implement SE2 Hybrid A-Star with Circular/Rectangular Footprints
- Implement Motion Model support for neighbor generation (in contrast with traditional motion primitives approaches)
- Support occupancy grid updating on the fly. Use a seperate thread to update our feasibility map
- Look into using Fibanacci Heap for faster decrease key operations in PQ in core A-star implementaion
- ROS1 and ROS2 bindings
