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

# 카메라 → 로봇 회전 행렬
R_cam_to_robot = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0]
])

# AMCL 전역 변수
X_R, Y_R, theta_amcl = 0.0, 0.0, 0.0

def quaternion_to_euler(q):
    return tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def local_to_global(x_local, y_local):
    x_world = X_R + x_local * np.cos(theta_amcl) - y_local * np.sin(theta_amcl)
    y_world = Y_R + x_local * np.sin(theta_amcl) + y_local * np.cos(theta_amcl)
    return x_world, y_world

class YOLODepthMultiNode:
    def __init__(self):
        rospy.init_node("yolo_multi_object_node")
        self.bridge = CvBridge()
        self.depth_scale = 0.001

        # 모델 로드
        self.model_wire = YOLO("best_wire.pt")
        self.model_coke = YOLO("best_coke_can.pt")

        # ROS 구독
        color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        self.sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        # AMCL pose 구독
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

        # 퍼블리셔
        self.wire_cam_pub = rospy.Publisher("/wire_camera_point", Point, queue_size=10)
        self.wire_robot_pub = rospy.Publisher("/wire_robot_point", Point, queue_size=10)
        self.coke_cam_pub = rospy.Publisher("/coke_camera_point", Point, queue_size=10)
        self.coke_robot_pub = rospy.Publisher("/coke_robot_point", Point, queue_size=10)

        rospy.loginfo("YOLO Depth Multi Object Detection Node Started")
        rospy.spin()

    def amcl_pose_callback(self, data):
        global X_R, Y_R, theta_amcl
        pos = data.pose.pose.position
        X_R, Y_R = pos.x, pos.y
        theta_amcl = quaternion_to_euler(data.pose.pose.orientation)

    def pixel_to_3d(self, cx, cy, depth):
        if depth is None or depth <= 0:
            return None, None, None
        Z = depth
        X = (cx - cx0) * (Z / fx)
        Y = (cy - cy0) * (Z / fy)
        return X, Y, Z

    def process_model(self, model, frame, depth_image, color, cam_pub, robot_pub):
        results = model(frame)[0]
        boxes = results.boxes
        masks = results.masks.data.cpu().numpy() if results.masks is not None else []

        for i, box in enumerate(boxes):
            if i >= len(masks): continue

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
                X, Y, Z = self.pixel_to_3d(x_px, y_px, best_z)
                if X is None: continue

                cam_pub.publish(Point(X, Y, Z))
                robot_coords = R_cam_to_robot @ np.array([X, Y, Z])
                x_r, y_r = robot_coords[0], robot_coords[1]
                x_w, y_w = local_to_global(x_r, y_r)
                robot_pub.publish(Point(x_w, y_w, 0.0))

                rospy.loginfo(f"[{model.model.names[0]}] Cam ({X:.2f}, {Y:.2f}, {Z:.2f}) → World ({x_w:.2f}, {y_w:.2f})")

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.circle(frame, (x_px, y_px), radius=4, color=color, thickness=-1)
                cv2.putText(frame, f"{model.model.names[0]} Z={Z:.2f}m", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    def callback(self, color_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        self.process_model(self.model_wire, frame, depth_image, (0, 255, 0), self.wire_cam_pub, self.wire_robot_pub)
        self.process_model(self.model_coke, frame, depth_image, (255, 0, 0), self.coke_cam_pub, self.coke_robot_pub)

        if not rospy.is_shutdown():
            cv2.imshow("YOLO Multi Object Detection", frame)
            cv2.waitKey(1)

if __name__ == "__main__":
    try:
        YOLODepthMultiNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
