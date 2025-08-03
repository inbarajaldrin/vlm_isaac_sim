import omni.ext
import omni.ui as ui
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading

class IsaacSimRosExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._window = ui.Window("Isaac Sim ROS Extension", width=300, height=200)
        self._received_data = ""
        self._ros_thread = None
        self._is_listening = False
        if hasattr(self, '_ros_node'):
            self._ros_node.destroy_node()
            self._ros_node = None
        self._topic_name = ""

        # Initialize ROS
        if not rclpy.ok():
            rclpy.init()

        with self._window.frame:
            with ui.VStack():
                self._topic_field = ui.StringField()
                self._topic_field.model.set_value("Enter topic name...")
                ui.Button("Listen to Topic", clicked_fn=self._on_connect_to_ros_clicked)
                self._data_field = ui.StringField()
                self._data_field.model.set_value("Data will appear here...")
                ui.Button("Stop Listening", clicked_fn=self._stop_listening)

    def _on_connect_to_ros_clicked(self):
        if not self._is_listening:
            self._is_listening = True
            self._topic_name = self._topic_field.model.get_value_as_string()
            self._ros_thread = threading.Thread(target=self._start_ros_node, daemon=True)
            self._ros_thread.start()
            self._update_data_field(f"Listening to topic: {self._topic_name}")

    def _stop_listening(self, force=True):
        self._is_listening = False
        if self._ros_thread and self._ros_thread.is_alive() and force:
            self._ros_thread.join()
            print("Stopped listening to the ROS topic.")

    def _start_ros_node(self):
        self._ros_node = JointStateListener(self._update_data_field, self._topic_name)
        node = self._ros_node
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

    def _update_data_field(self, data):
        # Update the UI text field with the received data
        if self._data_field:
            self._data_field.model.set_value(data)

    def on_shutdown(self):
        self._stop_listening(force=False)
        if rclpy.ok():
            rclpy.shutdown()
        if self._window:
            self._window.destroy()
        self._window = None

class JointStateListener(Node):
    def __init__(self, update_ui_callback, topic_name):
        super().__init__('joint_state_listener')
        self.update_ui_callback = update_ui_callback
        self.subscription = self.create_subscription(
            JointState,
            topic_name,
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        joint_data = ', '.join(f"{name}: {position:.4f}" for name, position in zip(msg.name, msg.position))
        print(f"Received joint states: {joint_data}")
        self.update_ui_callback(joint_data)
