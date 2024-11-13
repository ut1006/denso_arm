import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge
import cv2

# Initialize ROS node
rospy.init_node('yolo_segmentation_node')

# Path to your YOLO model
model_path = "/home/kamadagpu/catkin_ws/src/denso_arm/scripts/my_moveit/YOLO/roboflow/weight/best.pt"
model = YOLO(model_path)  # Load the YOLO model
bridge = CvBridge()  # To convert ROS Image messages to OpenCV images

def left_image_rect_callback(msg):
    # Convert ROS Image message to OpenCV format
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    
    # Run YOLO segmentation. set imgsz as 1120(= 32*35) for input resoltion 1104*621
    results = model.predict(source=frame, conf=0.25, iou=0.45, imgsz=1120)
    
    # Draw segmentation masks on the image
    for result in results:
        segmented_img = result.plot()  # Use .plot() for visualization

        # Display the segmented image
        cv2.imshow("YOLO Segmentation", segmented_img)
        cv2.waitKey(1)

# Set up ROS subscriber
rospy.Subscriber("/zedm/zed_node/left/image_rect_color", Image, left_image_rect_callback)

# Keep the script running
rospy.spin()

# Release resources on exit
cv2.destroyAllWindows()
