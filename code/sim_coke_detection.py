#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# YOLOv8 모델 로드
model = YOLO('/home/jeonjehwan/catkin_xarm/src/control/scripts/best_coke_can.pt')
bridge = CvBridge()


# 카메라 내부 파라미터 (수정 필요)
fx = 640.5098
fy = 640.5098
cx0 = 640.0
cy0 = 360.0

# 카메라 -> 월드 변환 행렬
R = np.array([[0, 0, 1],
              [1, 0, 0],
              [0, -1, 0]])

X_R, Y_R = 0.0, 0.0  # 로봇 위치 저장 변수

def amcl_pose_callback(data):
    global X_R, Y_R
    position = data.pose.pose.position
    X_R, Y_R = position.x, position.y
    rospy.loginfo(f"Robot Position: X={X_R}, Y={Y_R}")

def pixel_to_3d(cx, cy, depth):
    if depth is None or depth <= 0:
        return None, None, None
    Z = depth /1000.0 # mm -> meters 변환
    X = (cx - cx0) * (Z / fx)
    Y = (cy - cy0) * (Z / fy)
    return X, Y, Z

def callback(color_msg, depth_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "16UC1")
    except Exception as e:
        rospy.logerr(f"[ERROR] Image Conversion: {e}")
        return
    
    results = model(cv_image)
    min_z = float('inf')
    closest_object = None
    
    for result in results:
        if not result.boxes:
            continue

        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            label = result.names[int(box.cls[0])]
            conf = box.conf[0].item()
            Z = None
            if conf < 0.4:
                continue
            
            if 0 <= cx < depth_image.shape[1] and 0 <= cy< depth_image.shape[0]:
                z = depth_image[cy, cx]
            
                if z <= 0:
                    continue

                X, Y, Z = pixel_to_3d(cx, cy, z)
                rospy.loginfo(f"Detected {label} at ({cx}, {cy}, {z}) with confidence {conf:.2f}")
                rospy.loginfo(f"3D Position: ({X:.2f}, {Y:.2f}, {Z:.2f})")

                # 가장 가까운 객체 찾기
                if Z < min_z:
                    min_z = Z
                    closest_object = {"label": label, "x": X, "y": Y, "z": Z}
                    x1_close, y1_close = x1, y1
                elif Z == min_z and X < closest_object["x"]:
                    closest_object = {"label": label, "x": X, "y": Y, "z": Z}
                    x1_close, y1_close = x1, y1
                    
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, f'{label} {conf:.2f} Z:{z}', (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if Z is not None:
                cv2.putText(cv_image, f'({X:.2f}, {Y:.2f}, {Z:.2f}m)', 
                            (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)            
    
            
    rospy.loginfo(f"Closest object {closest_object}")

    
    if closest_object:
        closest_msg = Point()
        closest_msg.x = closest_object["x"]
        closest_msg.y = closest_object["y"]
        closest_msg.z = closest_object["z"]
        closest_pub.publish(closest_msg)

        cv2.putText(cv_image, 'Closest object', 
                    (x1_close, y1_close - 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
        world_coords = R @ np.array([closest_object["x"], closest_object["y"], closest_object["z"]])
           
        robot_msg = Point(world_coords[0] + X_R, world_coords[1] + Y_R, 0)
        closest_robot_pub.publish(robot_msg)
        rospy.loginfo(f"Closest Object in World: {robot_msg}")

    # OpenCV 창에 이미지 표시
    cv2.imshow("RealSense Camera - YOLOv8", cv_image)

    # 'q' 키를 눌렀을 때 창 닫기
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Closed by user")


def main():
    rospy.init_node('realsense_yolo_odom', anonymous=True)
    
    # 퍼블리셔를 init_node() 이후에 정의
    global closest_pub, closest_robot_pub
    closest_pub = rospy.Publisher('/3D_closest_coordinate', Point, queue_size=10)
    closest_robot_pub = rospy.Publisher('/3D_closest_robot_coordinate', Point, queue_size=10)

    rospy.Subscriber("/amcl_pose",  PoseWithCovarianceStamped, amcl_pose_callback)

    global closest_pub, closest_robot_pub
    color_sub = message_filters.Subscriber("realsense_gazebo_camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/realsense_gazebo_camera/aligned_depth_to_color/image_raw", Image)
    sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
    sync.registerCallback(callback)
    
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
