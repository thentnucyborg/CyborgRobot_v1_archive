# ntnu_cyborg_follower

`ntnu_cyborg_follower` is a ROS catkin package for the NTNU Cyborg project. It implements a follower functionality where the robot will follow a person using sensor data from a Kinect sensor via the `k2_client` package.

The following packages must be available to build `ntnu_cyborg_follower`:

* https://github.com/thentnucyborg/ntnu_cyborg_coordinator
* https://github.com/thentnucyborg/k2_client

## Launching the follower in standalone mode

The follower can be run by itself with only the `k2_client` dependency using:

`roslaunch ntnu_cyborg_follower standalone`

This command will launch `k2_client`, and the `ntnu_cyborg_follower` node with the `use_coordinator` parameter set to `false`.

## Launching the follower using the coordinator

When running the follower together with e.g. `ntnu_cyborg_simon_says`, coordination is necessary to prevent the robot from starting a game of Simon Says while following a person. This is achieved by negotiating robot control using `ntnu_cyborg_coordinator` as a mediator. After `ntnu_cyborg_coordinator` and `k2_client` has been launched, the follower can be launched using:

`roslaunch ntnu_cyborg_follower use_coordinator`

This command will launch the `ntnu_cyborg_follower` node with the `use_coordinator` parameter set to `true`.

The `ntnu_cyborg_follower` node will now gain control over the robot from the coordinator before following a person, and release control of the robot after following a person. See the following for more information:

https://github.com/thentnucyborg/ntnu_cyborg_coordinator

## Making the robot follow you

When the follower is launched, you can make the robot follow you by putting one of your arms straight out to the side. The robot will follow you until you no longer have your arm out to the side.

## References

* http://www.ros.org/
* https://www.ntnu.edu/cyborg/student-organization/
