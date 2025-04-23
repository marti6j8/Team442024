import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

class YOLORealSenseNode(Node):
    def __init__(self):
        super().__init__("yolo_realsense_node")
        self.model = YOLO("/home/iras/ros2_ws/src/Roboflow_YOLOV11/CubeFacesV2/runs/detect/train/weights/DetectCubeFacesV2.pt")
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.camera_info_callback, 10)
        self.target_pub = self.create_publisher(PointStamped, "/yolo/target_point", 10)

        self.latest_depth_image = None
        self.camera_intrinsics = None
        self.image_height = 480
        self.last_valid_z = None

        # Define rotation and translation from camera frame to robot base frame
        original_rotation = np.array([
            [-0.5322, -0.6999,  0.4764],
            [-0.5809, -0.1075, -0.8068],
            [ 0.6159, -0.7061, -0.3494]
        ])
        original_rotation[:, 1] *= -1  # Optional flip if needed
        self.rotation_matrix = original_rotation
        self.translation_vector = np.array([0.1841, 0.7284, 0.3566])

        self.get_logger().info("YOLOv11 RealSense Node Started")

    def camera_info_callback(self, msg):
        self.camera_intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }
        self.image_height = msg.height

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def pixel_to_real_world(self, cx, cy, depth):
        if self.camera_intrinsics is None:
            return None, None, None
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx_0 = self.camera_intrinsics['cx']
        cy_0 = self.camera_intrinsics['cy']
        X_real = (cx - cx_0) * depth / fx
        Y_real = (cy - cy_0) * depth / fy
        Z_real = depth
        return X_real, Y_real, Z_real

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        results = self.model(frame)
        if results[0].boxes is not None:
            top_face = None
            top_y = float('inf')

            for box in results[0].boxes.xyxy:
                x1, y1, x2, y2 = map(int, box[:4])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                if y1 < top_y:
                    top_face = (cx, cy)
                    top_y = y1

            if top_face:
                cx, cy = top_face
                z_value = 0.0

                if self.latest_depth_image is not None:
                    try:
                        z_value = float(self.latest_depth_image[cy, cx]) / 1000.0
                        if z_value > 0:
                            self.last_valid_z = z_value
                        elif self.last_valid_z is not None:
                            z_value = self.last_valid_z
                        else:
                            z_value = 0.0
                    except Exception as e:
                        self.get_logger().warn(f"Depth lookup failed at ({cx}, {cy}): {e}")
                        z_value = self.last_valid_z if self.last_valid_z is not None else 0.0

                X_real, Y_real, Z_real = self.pixel_to_real_world(cx, cy, z_value)
                if X_real is not None and Y_real is not None and Z_real is not None:
                    camera_point = np.array([X_real, Y_real, Z_real])
                    robot_point = self.rotation_matrix @ camera_point + self.translation_vector
                    X_robot, Y_robot, Z_robot = robot_point.tolist()

                    target_msg = PointStamped()
                    target_msg.header.stamp = self.get_clock().now().to_msg()
                    target_msg.header.frame_id = "base_link"
                    target_msg.point.x = X_robot
                    target_msg.point.y = Y_robot
                    target_msg.point.z = Z_robot
                    self.target_pub.publish(target_msg)

                    self.get_logger().info(f"Detected Target in Robot Frame: X={X_robot:.3f}, Y={Y_robot:.3f}, Z={Z_robot:.3f}")
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                    label = f"({X_robot:.3f}, {Y_robot:.3f}, {Z_robot:.3f}) m"
                    cv2.putText(frame, label, (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("YOLO RealSense Stream", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YOLORealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


