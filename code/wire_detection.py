#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import tf.transformations as tf_trans

# 카메라 내부 파라미터
fx = 385.693359375
fy = 385.1759948730469
cx0 = 322.4962158203125
cy0 = 244.2171173095703

# 카메라 → 로봇 기준 회전 행렬
R_cam_to_robot = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0]
])

# 전역 변수 (AMCL 로봇 위치)
X_R, Y_R, theta_amcl = 0.0, 0.0, 0.0

def quaternion_to_euler(q):
    """쿼터니언 → 오일러 변환 (yaw만 사용)"""
    return tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def amcl_pose_callback(data):
    global X_R, Y_R, theta_amcl
    pos = data.pose.pose.position
    X_R, Y_R = pos.x, pos.y
    theta_amcl = quaternion_to_euler(data.pose.pose.orientation)
    rospy.loginfo(f"[AMCL] Robot pose: X={X_R:.2f}, Y={Y_R:.2f}, θ={theta_amcl:.2f} rad")

def local_to_global(x_local, y_local):
    """로컬 좌표 → 전역 좌표계"""
    x_world = X_R + x_local * np.cos(theta_amcl) - y_local * np.sin(theta_amcl)
    y_world = Y_R + x_local * np.sin(theta_amcl) + y_local * np.cos(theta_amcl)
    return x_world, y_world

def pixel_to_3d(cx, cy, depth):
    """픽셀 + depth → 카메라 기준 3D 좌표"""
    if depth is None or depth <= 0:
        return None, None, None
    Z = depth  # 이미 m 단위
    X = (cx - cx0) * (Z / fx)
    Y = (cy - cy0) * (Z / fy)
    return X, Y, Z

class YOLODepthNode:
    def __init__(self):
        rospy.init_node("yolo_depth_ros_node")
        self.bridge = CvBridge()
        self.model = YOLO("best_wire.pt")
        self.depth_scale = 0.001

        # ROS 구독
        color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        # AMCL 위치 수신
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

        # 퍼블리셔
        self.cam_point_pub = rospy.Publisher("/camera_point", Point, queue_size=10)
        self.robot_point_pub = rospy.Publisher("/robot_point", Point, queue_size=10)

        rospy.loginfo("YOLODepthNode with robot coordinate transform initialized.")
        rospy.spin()

    def callback(self, color_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        results = self.model(frame)[0]

        boxes = results.boxes
        masks = results.masks.data.cpu().numpy() if results.masks is not None else []

        for i, box in enumerate(boxes):
            if i >= len(masks):
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            mask = masks[i]
            mask_resized = cv2.resize(mask.astype(np.uint8), (frame.shape[1], frame.shape[0]))
            mask_bool = mask_resized.astype(bool)

            x_center = (x1 + x2) // 2
            best_z = float('inf')
            best_px = None

            if 0 <= x_center < frame.shape[1]:
                y_candidates = np.where(mask_bool[y1:y2, x_center])[0]

                for offset in y_candidates:
                    y = y1 + offset
                    if 0 <= y < depth_image.shape[0]:
                        z_raw = depth_image[y, x_center]
                        z = z_raw * self.depth_scale
                        if 0.01 < z < 3.0 and z < best_z:
                            best_z = z
                            best_px = (x_center, y)

            if best_px:
                x_px, y_px = best_px
                X, Y, Z = pixel_to_3d(x_px, y_px, best_z)

                # 카메라 기준 퍼블리시
                cam_msg = Point(x=X, y=Y, z=Z)
                self.cam_point_pub.publish(cam_msg)

                # 로봇 기준 좌표계 변환
                robot_coords = R_cam_to_robot @ np.array([X, Y, Z])
                x_r, y_r = robot_coords[0], robot_coords[1]

                # 전역 좌표로 변환
                x_world, y_world = local_to_global(x_r, y_r)
                robot_msg = Point(x=x_world, y=y_world, z=0.0)
                self.robot_point_pub.publish(robot_msg)

                # 디버그 출력
                rospy.loginfo(f"[Camera] (u,v)=({x_px},{y_px}) → (X,Y,Z)=({X:.2f},{Y:.2f},{Z:.2f})")
                rospy.loginfo(f"[Robot] Local ({x_r:.2f},{y_r:.2f}) → Global ({x_world:.2f},{y_world:.2f})")

                # 화면 출력
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f"(u,v)=({x_px},{y_px})", (x1, y1 - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, f"(X,Y,Z)=({X:.2f},{Y:.2f},{Z:.2f})", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # 시각화
        if not rospy.is_shutdown():
            cv2.imshow("YOLO + Depth + Robot Frame", frame)
            cv2.waitKey(1)

if __name__ == "__main__":
    try:
        YOLODepthNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
