from .utils import CameraExecutor, CameraNode
import rclpy

def main(args=None):
    rclpy.init(args=args)
    try:
        node1 = CameraNode(0)
        node2 = CameraNode(1)

        executor = CameraExecutor()
        executor.add_node(node1)
        executor.add_node(node2)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node1.destroy_node()
            node2.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()