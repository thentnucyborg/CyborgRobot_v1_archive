# ntnu_cyborg_simon_says

`ntnu_cyborg_simon_says` is a ROS catkin package for the NTNU Cyborg project. It implements a single-player version of the Simon Says game using sensor data from a Kinect sensor via the `k2_client` package, and outputs speech using the `trollnode` package.

The following packages must be available to run `ntnu_cyborg_simon_says`:

* https://github.com/thentnucyborg/k2_client/
* https://github.com/thentnucyborg/trollnode/

## Launching the game in standalone mode

The Simon Says game can be run by itself with only the `k2_client` and `trollnode` dependencies using:

`roslaunch ntnu_cyborg_simon_says standalone`

This command will launch `k2_client`, the `setExpression` node from the `trollnode` package, and the `ntnu_cyborg_simon_says` node with the `use_coordinator` parameter set to `false`.

## Launching the game using the coordinator

When running the game together with e.g. `ntnu_cyborg_follower`, coordination is necessary to prevent the robot from moving while playing the Simon Says game. This is achieved by negotiating robot control using `ntnu_cyborg_coordinator` as a mediator. After `ntnu_cyborg_coordinator` and `k2_client` has been launched, the Simon Says game can be launched using:

`roslaunch ntnu_cyborg_simon_says use_coordinator`

This command will launch the `setExpression` node from the `trollnode` package and the `ntnu_cyborg_simon_says` node with the `use_coordinator` parameter set to `true`.

The `ntnu_cyborg_simon_says` node will now gain control over the robot from the coordinator before a game can be started, and release control of the robot after the game has been completed. See the following for more information:

https://github.com/thentnucyborg/ntnu_cyborg_coordinator

## Troll Speech

Speech output is produced by the `trollface` application. The `setExpression` node from the `trollnode` package forwards text via TCP/IP to the Troll server. This application must be run separately to enable speech output. See the following for more information:

https://www.ntnu.no/wiki/display/cyborg/e.+Trollface

The speech messages sent from the game can be viewed by running:

`rostopic echo /trollExpression`

## Playing the game

When the game is launched, a game can be started by waving to the robot using your right hand. When the game is started, the robot will welcome you to the game through the Troll's voice.

## References

* https://en.wikipedia.org/wiki/Simon_Says
* http://www.ros.org/
* https://www.ntnu.edu/cyborg/student-organization/
