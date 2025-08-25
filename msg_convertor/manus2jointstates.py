import time, rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState 
from manus_ros2_msgs.msg import ManusGlove
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class Manus2JointStates(Node):
    def __init__(self):
        self.glove_msg = None
        self.dt = None
        self.last_time = time.time()

        super().__init__('manus_to_jointstates_msg_convertor')

        # --- Declare & read parameters ---
        self.declare_parameter('indices', '')
        self.declare_parameter('timer_period', 0.01)       # sec
        self.declare_parameter('glove_topic', 'manus_glove_0')
        self.declare_parameter('out_topic', 'action')
        self.declare_parameter('highlight', True)

        raw_indices = self.get_parameter('indices').value
        self.indices_set = set(self._parse_indices_param(raw_indices))

        timer_period = float(self.get_parameter('timer_period').value)
        glove_topic = str(self.get_parameter('glove_topic').value)
        out_topic = str(self.get_parameter('out_topic').value)

        # --- ROS I/O ---
        self.publisher_ = self.create_publisher(JointState, out_topic, 10)
        self.subscription = self.create_subscription(ManusGlove, glove_topic, self.manusGlove_callback, 10)
        self.ui_timer = self.create_timer(0.5, self.ui_callback)
        self.main_timer = self.create_timer(timer_period, self.loop)

    def _parse_indices_param(self, raw):
        if isinstance(raw, list):             
            return [int(x) for x in raw]
        if isinstance(raw, str):
            s = raw.strip()
            if s.startswith('[') and s.endswith(']'):
                s = s[1:-1]
            if not s:
                return []
            return [int(p.strip()) for p in s.split(',') if p.strip()]
        return []
    
    def ui_callback(self):
        try:
            print("\033[2J\033[H", end="", flush=True)
            print(f"[loop] dt = {self.dt:.4f} sec \n")

            if self.glove_msg is None:
                print("no msg")
                return
            
            self.renderUi(self.glove_msg)

        except Exception:
            pass

    def loop(self):
        try:
            if self.glove_msg is None:
                return
        
            now = time.time()
            self.dt = now - self.last_time
            self.last_time = now
            
            js = self.convertGloveMsgToJointState(self.glove_msg)

            self.publisher_.publish(js)
        
        except Exception:
            pass

    def manusGlove_callback(self, msg):
        self.glove_msg = msg

    def renderUi(self, msg):        
        RESET = "\033[0m"
        HIGHLIGHT = "\033[7m" # White BackGround

        print("=== ManusGlove Live View ===")
        print("glove_id   :", msg.glove_id)
        print("side       :", msg.side)
        print("raw_nodes  :", len(msg.raw_nodes))
        print("ergonomics :", len(msg.ergonomics))

        print("\n-- Ergonomics Detail --")
        for i, e in enumerate(msg.ergonomics):
            line = f"{i:2d} {e.type:20s} : {e.value:.3f}"

            if i in self.indices_set:
                print(f"{HIGHLIGHT}{line}{RESET}")
            else:
                print(line)

    def convertGloveMsgToJointState(self, msg):
        jointstate = JointState()
        jointstate.header.stamp = self.get_clock().now().to_msg()

        for i, e in enumerate(msg.ergonomics):
            if i in self.indices_set:
                jointstate.name.append(e.type) 
                jointstate.position.append(e.value)

        return jointstate

def main(args=None):
    rclpy.init(args=args)
    node = Manus2JointStates()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
