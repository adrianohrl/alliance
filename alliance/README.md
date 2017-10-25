# alliance

This is a ROS package that implements the ALLIANCE[1][parker1998alliance] approach for Multi Robot Task Allocation architecture.

This package has two nodes: *low_level* and *high_level*. Each robot of the system must run in order to this architecture can run correctly.

The [*alliance_test*](../blob/master/mrta_archs/alliance_test) package shows how to use and configure each node for a given MRTA problem and a cooperative multirobot team (MRS).

## Future works

Usage of the diagnostic package for checking the rate at which a topic is publishing, or checking that timestamps are sufficiently recent.

## References

[parker1998alliance]: **Parker, Lynne E**. "ALLIANCE: An architecture for fault tolerant multirobot cooperation." *IEEE transactions on robotics and automation* 14.2 (1998): 220-240.