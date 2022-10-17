#!/usr/bin/env python
# TODO: Figure out ROS2 equivalent of diagnostic_updater

import traceback
import threading

# import diagnostic_updater
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

# import diagnostic_msgs

from roboclaw_interfaces.msg import SpeedCommand, Stats
from roboclaw_driver2.roboclaw_control import RoboclawControl
from roboclaw_driver2.roboclaw import Roboclaw
from roboclaw_driver2.roboclaw_stub import RoboclawStub


DEFAULT_DEV_NAMES = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200
DEFAULT_NODE_NAME = "roboclaw_driver"
DEFAULT_LOOP_HZ = 10
DEFAULT_ADDRESS = 0x80
DEFAULT_DEADMAN_SEC = 3
DEFAULT_STATS_TOPIC = "stats"
DEFAULT_SPEED_CMD_TOPIC = "speed_command"

PARAM_STATS_TOPIC = 'stats_topic'
PARAM_SPEED_CMD_TOPIC = 'speed_cmd_topic'
PARAM_DEV_NAMES = 'dev_names'
PARAM_BAUD_RATE = 'baud_rate'
PARAM_ADDRESS = 'address'
PARAM_LOOP_HZ = 'loop_hz'
PARAM_DEADMAN_SECS = 'deadman_secs'
PARAM_TEST_MODE = 'test_mode'


class RoboclawNode(Node):
    def __init__(self):
        """
        Parameters:
            :param str node_name: ROS node name
            :param RoboclawControl rbc_ctl: RoboclawControl object that controls the hardware
            :param int loop_rate: Integer rate in Hz of the main loop
        """
        super().__init__(DEFAULT_NODE_NAME)

        self.declare_parameters(
            namespace='',
            parameters=[
                (PARAM_STATS_TOPIC, DEFAULT_STATS_TOPIC, ParameterDescriptor(description='Topic on which to publish Robloclaw stats')),
                (PARAM_SPEED_CMD_TOPIC, DEFAULT_SPEED_CMD_TOPIC, ParameterDescriptor(description='Topic on which to listen for SpeedCommands')),
                (PARAM_DEV_NAMES, DEFAULT_DEV_NAMES, ParameterDescriptor(description='OS path for the Roboclaw devices (comma-separated)')),
                (PARAM_BAUD_RATE, DEFAULT_BAUD_RATE, ParameterDescriptor(description='Roboclaw serial baud rate')),
                (PARAM_ADDRESS, DEFAULT_ADDRESS, ParameterDescriptor(description='Roboclaw serial address')),
                (PARAM_LOOP_HZ, DEFAULT_LOOP_HZ, ParameterDescriptor(description='Node main loop frequency in hertz')),
                (PARAM_DEADMAN_SECS, DEFAULT_DEADMAN_SEC, ParameterDescriptor(description='Max seconds before Roboclaw stops witout a command')),
                (PARAM_TEST_MODE, False, ParameterDescriptor(description='Simulate Roboclaw hardware'))
            ]
        )

        self._rbc_ctls = []  # Populated by the connect() method

        # Records the values of the last speed command
        self._last_cmd_time = self.get_clock().now()
        self._last_cmd_m1_qpps = 0
        self._last_cmd_m2_qpps = 0
        self._last_cmd_accel = 0
        self._last_cmd_max_secs = 0
        self._speed_cmd_lock = threading.RLock()  # To serialize access to cmd variables

        self._deadman_secs = self.get_parameter(PARAM_DEADMAN_SECS).value

        # Set up the Publishers
        self.stats_pub = self.create_publisher(
            msg_type=Stats,
            topic=self.get_parameter(PARAM_STATS_TOPIC).value,
            qos_profile=1
        )

        # Set up the Diagnostic Updater
        # self._diag_updater = diagnostic_updater.Updater()
        # self._diag_updater.setHardwareID(node_name)
        # self._diag_updater.add("Read Diagnostics", self._publish_diagnostics)

        # Set up the SpeedCommand Subscriber
        self.create_subscription(
            msg_type=SpeedCommand,
            topic=self.get_parameter(PARAM_SPEED_CMD_TOPIC).value,
            callback=self._speed_cmd_callback,
            qos_profile=1
        )

        # For logdebug
        self.prev_m1_val = 0
        self.prev_m2_val = 0

        # Main loop timer
        loop_secs = 1.0 / self.get_parameter(PARAM_LOOP_HZ).value
        self.create_timer(loop_secs, self._main_loop)

    @property
    def roboclaw_control(self):
        return self._rbc_ctl

    def connect(self, dev_name: str, baud_rate: int, address: int, test_mode: bool = False):
        """Connects the node to the Roboclaw controller, or the test stub.

        Args:
            dev_name (str): Serial device name (e.g. /dev/ttyACM0)
            baud_rate (int): Serial baud rate (e.g. 115200)
            address (int): Serial address (default 0x80)
            test_mode (bool, optional): True if connecting to the controller test stub. Defaults to False.
        """
        self.get_logger().info("Connecting to roboclaw")
        if not test_mode:
            roboclaw = Roboclaw(dev_name, baud_rate)
        else:
            self.get_logger().info('Running in test mode. Connecting to stubbed Roboclaw device')
            roboclaw = RoboclawStub(dev_name, baud_rate)
        self._rbc_ctls.append(RoboclawControl(roboclaw, address))

    def _main_loop(self):
        """Callback run on a timer to perform periodic node functions:
        - Read encoders from the roboclaws and publish on the stats topic
        - Stop the node if no command has been received in < deadman_secs
        """
        # Read encoder readings
        read_success, stats = self._rbc_ctls[0].read_stats()
        for error in stats.error_messages:
            self.get_logger().warn(error)

        # Publish the encoder readings as stats
        if read_success:
            msg = Stats()

            msg.header.stamp = self.get_clock().now().to_msg()

            msg.m1_enc_val = stats.m1_enc_val
            msg.m1_enc_qpps = stats.m1_enc_qpps

            msg.m2_enc_val = stats.m2_enc_val
            msg.m2_enc_qpps = stats.m2_enc_qpps

            self.get_logger().debug((
                f"Encoder diffs M1:{stats.m1_enc_val - self.prev_m1_val},"
                f" M2:{stats.m2_enc_val - self.prev_m2_val}"
            ))
            self.prev_m1_val = stats.m1_enc_val
            self.prev_m2_val = stats.m2_enc_val

            self.stats_pub.publish(msg)
        else:
            self.get_logger.warn("Error reading stats from Roboclaw: {stats}")

        # Stop motors if running and no commands are being received
        if (stats.m1_enc_qpps != 0 or stats.m2_enc_qpps != 0):
            if (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9 > self._deadman_secs:
                self.get_logger().info("Did not receive a command for over 1 sec: Stopping motors")
                decel = max(abs(stats.m1_enc_qpps), abs(stats.m2_enc_qpps)) * 2
                for rbc_ctl in self._rbc_ctls:
                    rbc_ctl.stop(decel=decel)

                # Publish diagnostics
                # self._diag_updater.update()

    # def _publish_diagnostics(self, stat):
    #     """Function called by the diagnostic_updater to fetch and publish diagnostics
    #     from the Roboclaw controller

    #     Parameters:
    #     :param diagnostic_updater.DiagnosticStatusWrapper stat:
    #         DiagnosticStatusWrapper provided by diagnostic_updater when called

    #     Returns: The updated DiagnosticStatusWrapper
    #     :rtype: diagnostic_updater.DiagnosticStatusWrapper
    #     """
    #     for i, rbc_ctl in enumerate(self._rbc_ctls):
    #         diag = rbc_ctl.read_diag()

    #         stat.add("[{}] Temperature 1 (C):".format(i), diag.temp1)
    #         stat.add("[{}] Temperature 2 (C):".format(i), diag.temp2)
    #         stat.add("[{}] Main Battery (V):".format(i), diag.main_battery_v)
    #         stat.add("[{}] Logic Battery (V):".format(i), diag.logic_battery_v)
    #         stat.add("[{}] Motor 1 current (Amps):".format(i), diag.m1_current)
    #         stat.add("[{}] Motor 2 current (Amps):".format(i), diag.m2_current)

    #         for msg in diag.error_messages:
    #             level = diagnostic_msgs.msg.DiagnosticStatus.WARN
    #             if "error" in msg:
    #                 level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
    #             stat.summary(level, "[{}]: msg".format(i))

    #     return stat

    def _speed_cmd_callback(self, command: SpeedCommand):
        """Callback for processing messages from the SpeedCommand subscriber.

        Args:
            command (SpeedCommand): The forward/turn command message
        """
        with self._speed_cmd_lock:
            self._last_cmd_time = self.get_clock().now()

            # Skip if the new command is not different than the last command
            if (command.m1_qpps == self._last_cmd_m1_qpps
                and command.m2_qpps == self._last_cmd_m2_qpps
                and command.accel == self._last_cmd_accel
                and command.max_secs == self._last_cmd_max_secs
            ):
                self.get_logger().debug("Speed Command received, but no change in command values")

            else:
                self.get_logger().debug((
                    f"M1 speed: {command.m1_qpps} | M2 speed: {command.m2_qpps} |"
                    f" Accel: {command.accel} | Max Secs: {command.max_secs}"
                ))

                for rbc_ctl in self._rbc_ctls:
                    success = rbc_ctl.driveM1M2qpps(
                        command.m1_qpps, command.m2_qpps,
                        command.accel, command.max_secs
                    )

                    if not success:
                        self.get_logger().error("RoboclawControl SpeedAccelDistanceM1M2 failed")


def main(args=None):

    # Setup the ROS node
    rclpy.init(args=args)
    node = RoboclawNode()

    # Read the input parameters
    dev_names = node.get_parameter(PARAM_DEV_NAMES).value
    baud_rate = node.get_parameter(PARAM_BAUD_RATE).value
    address = node.get_parameter(PARAM_ADDRESS).value
    test_mode = node.get_parameter(PARAM_TEST_MODE).value
    deadman_secs = node.get_parameter(PARAM_DEADMAN_SECS).value
    loop_hz = node.get_parameter(PARAM_LOOP_HZ).value

    node.get_logger().debug(f"node_name: {node.get_name()}")
    node.get_logger().debug(f"dev_names: {dev_names}")
    node.get_logger().debug(f"baud: {baud_rate}")
    node.get_logger().debug(f"address: {address}")
    node.get_logger().debug(f"loop_hz: {loop_hz}")
    node.get_logger().debug(f"deadman_secs: {deadman_secs}")
    node.get_logger().debug(f"test_mode: {test_mode}")

    try:
        # Initialize the Roboclaw controllers
        for dev in dev_names.split(','):
            node.connect(dev, baud_rate, address, test_mode)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().fatal("Unhandled exeption...printing stack trace then shutting down node")
        node.get_logger().fatal(traceback.format_exc())

    # Shutdown and cleanup
    rclpy.shutdown()
