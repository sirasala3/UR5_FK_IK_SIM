import sys
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ur5_controller import UR5JazzyController

def main(args=None):
    rclpy.init(args=args)
    
    controller = UR5JazzyController()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        def demo_thread():
            # Wait for ROS to be ready
            time.sleep(1)
            controller.run_demo()

        t = threading.Thread(target=demo_thread)
        t.daemon = True
        t.start()

        # Spin the executor to process all callbacks (timers, etc.)
        executor.spin()

    except KeyboardInterrupt:
        print("\nShutdown requested...")
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

    return 0

if __name__ == "__main__":
    sys.exit(main())

