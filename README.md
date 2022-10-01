# `roboclaw_driver2`

A ROS2 node providing a driver interface to the Roboclaw motor controller.

---
This is the ROS2 version of the previous repo which was build for ROS1 ([sheaffej/roboclaw_driver](https://github.com/sheaffej/roboclaw_driver)).

## Why another ROS Roboclaw driver?
There are several existing ROS nodes for the roboclaw, including:

* [https://github.com/sonyccd/roboclaw_ros](https://github.com/sonyccd/roboclaw_ros)
* [https://github.com/doisyg/roboclaw_ros](https://github.com/doisyg/roboclaw_ros)
* [https://github.com/SV-ROS/roboclaw_driver](https://github.com/SV-ROS/roboclaw_driver)

All three follow a similar approach, where the Roboclaw node is essentially the 2-wheel differential drive base (aka base_link). Therefore, the Roboclaw node in these repositories compute and publish the Odometry and tf frame data.

However I am buidling a 4-wheel differential drive base that drives two Roboclaw controllers. I have a separate base node that sends commands to two separate Roboclaw driver nodes. Therefore this roboclaw driver node needed to be more of a ROS wrapper to the Roboclaw controller. 

In my robot, the Odometry and tf frames will be computed and published by the separate base node, using encoder readings published by the Roboclaw nodes as well as fused with IMU sensor data to improve the Odometry accuracy.

## Parameters

* `dev_name` - Serial (aka USB) device name
* `baud` - Default 115200
* `address` - Serial address, default 0x80
* `loop_hz` - Number of publisher loops per second (Hertz)
* `deadman_secs` - Seconds until motors stop without additional commands
* `test_mode` - True = run the Roboclaw simulator stub for testing
* `speed_cmd_topic` - Topic on which to listen for SpeedCommand messages
* `stats_topic` - Topic on which Stats messages will be published

## Topics

### Publishes:

Stats topic: `/roboclaw_stats`

* Motor 1 & 2 encoder values
* Motor 1 & 2 speed values in QPPS (+/-)

<!-- Diagnostic updater topic: `/diagnostics`

* Temperature 1 (C)
* Temperature 2 (C)
* Main Battery (V)
* Logic Battery (V)
* Motor 1 current (Amps)
* Motor 2 current (Amps) -->

#### Subscribes to:
SpeedCommand topic: `/speed_command`

* M1 and M2 motor speed in QPPS
* Max seconds before automatically stopping

## Launching
Clone the repository, and the associated `roboclaw_interfaces` package

```
cd $ROS_WS/src
git clone https://github.com/sheaffej/roboclaw_driver2.git
git clone https://github.com/sheaffej/roboclaw_interfaces.git
```

Build the packages, also creating the message bindings

```
cd $ROS_WS
rosdep install --from-paths src -y
colcon build
```

Launch the node to use with a Roboclaw device

```
ros2 launch roboclaw_driver2 roboclaw_node
```

Optionally, launch the node with the simulated Roboclaw device (a.k.a. `roboclaw_stub.py`)
```
ros2 launch roboclaw_driver2 roboclaw_node test_mode:=true
```

## Tests
The script `tests/run_tests.sh` is a helper script to manually run the unit tests using `pytest`. Both regular unit tests (`test/unit/`), and larger node-level tests (`test/node/`) are run using `pytest`,

### Unit tests
The only logic that is non-trivial and therefore likely to break during refactoring is the RoboclawStub object that simulates the hardware controller for use in testing. The rest of the logic is more of a wrapper, and therefore will be tested during node-level integration testing.

```
pytest src/test_roboclaw_stub_unit.py
```

### Node-level integration tests
These are performed by running at test_node and the roboclaw_node in the same ROS2 context, and explicitly advancing the executor to process through the inter-node messages. These node tests use the simulated roboclaw controller (RoboclawStub)

```
pytest src/test_roboclaw_node.py
```

## Attributions
The `roboclaw.py` library is every so slightly modified (basic formatting and comments) from the version downloadable from the Ion Motion control site :
[http://downloads.ionmc.com/code/roboclaw_python.zip]()
