import rclpy
import rclpy.node
import rclpy.executors
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, BatteryState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from diagnostic_msgs.msg import DiagnosticArray
from threading import Event, Thread


class MockSubscriberNode(rclpy.node.Node):
    """
    A common test node to subscribe to topics and wait for them to be published.
    Useful for validating topic rates and latched logic.
    """
    def __init__(self, name='mock_subscriber_node'):
        super().__init__(name)
        self.events = {
            'battery': Event(),
            'mode': Event(),
            'joint_states': Event(),
            'is_homed': Event(),
            'is_runstopped': Event(),
            'odom': Event(),
            'tool': Event(),
            'tf': Event(),
            'diagnostics': Event()
        }
        self.msg_counts = {k: 0 for k in self.events.keys()}
        self.latest_msgs = {k: None for k in self.events.keys()}

        self.create_subscription(BatteryState, 'battery', lambda msg: self._cb('battery', msg), 10)
        self.create_subscription(String, 'mode', lambda msg: self._cb('mode', msg), 10)
        self.create_subscription(JointState, '/stretch/joint_states', lambda msg: self._cb('joint_states', msg), 10)
        self.create_subscription(Bool, '/is_homed', lambda msg: self._cb('is_homed', msg), 10)
        self.create_subscription(Bool, '/is_runstopped', lambda msg: self._cb('is_runstopped', msg), 10)
        self.create_subscription(Odometry, '/wheel_odom', lambda msg: self._cb('odom', msg), 10)
        self.create_subscription(String, '/tool', lambda msg: self._cb('tool', msg), 10)
        self.create_subscription(TFMessage, '/tf', lambda msg: self._cb('tf', msg), 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', lambda msg: self._cb('diagnostics', msg), 10)

        # Setup background spinning
        self._spin_thread_shutdown_flag = Event()
        self._spin_thread = Thread(
            target=self._spinner,
            args=(self, rclpy.executors.SingleThreadedExecutor(),),
            daemon=True,
        )
        self._spin_thread.start()

    def _spinner(self, node, executor):
        try:
            while not self._spin_thread_shutdown_flag.is_set():
                rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
        except rclpy.executors.ExternalShutdownException:
            pass

    def _cb(self, key, msg):
        self.events[key].set()
        self.msg_counts[key] += 1
        self.latest_msgs[key] = msg

    def wait_for_message(self, topic_key, timeout=5.0):
        self.events[topic_key].clear()
        return self.events[topic_key].wait(timeout=timeout)

    def stop(self):
        self._spin_thread_shutdown_flag.set()
        self._spin_thread.join()
        self.destroy_node()
