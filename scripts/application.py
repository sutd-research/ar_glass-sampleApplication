#!/usr/bin/python
import rospy
import sys
import signal
from ar_glass.srv import Image as ImageSrv
from ar_glass.msg import BoundingBox
from ar_glass.srv import ImageRequest, ImageResponse
from sensor_msgs.msg import Image as ImageMsg 
from cv_bridge import CvBridge
import cv2

request_service = None
bounding_box_publisher = None
bridge = CvBridge()
header = None

def process_image(image):
    """
        User function to process the image received from AR Glass
        :param image:   Received image from AR Glass (BGR Color Encoding)
        :return     bounding box coordinates
    """
    ##################################################################
    ######################### Example User Code ######################
    # processed_image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE) 
    boundingBox = (150, 50, 50, 150)
    ##################################################################
    return boundingBox

def send_bounding_box(coordinates):
    """
        Sends a bounding box to AR Glass Driver
        :param coordinates: (leftx, topy, rightx, bottomy)
    """

    box = BoundingBox()
    box.left_x, box.top_y, box.right_x, box.bottom_y = coordinates

    box.header = header
    bounding_box_publisher.publish(box)

def get_image():
    """
        Requests and image from AR Glass Driver
        :return     image 
        
        :Throws     ServiceException
    """
    request = ImageRequest()
    response = request_service(request)
    
    # Saving received image header data (time, sequence number, frame ID), 
    # to be used when sending back the image. This is done enable correspondance check
    global header
    header = response.image.header

    return bridge.imgmsg_to_cv2(response.image, "bgr8")


def init():
    # ROS Node Initialization
    rospy.init_node('user_application')
    signal.signal(signal.SIGINT, terminate)

    # Reading loaded package parameters (Defined in parameters.yaml)
    request_service_name = rospy.get_param(rospy.get_name()+'/image_request_service', 'AR_Take_Image')
    image_topic_name = rospy.get_param(rospy.get_name()+"/image_publisher_topic", 'AR_Send_Image')

    # Registering Service Client
    global request_service, bounding_box_publisher
    try:
        # Check if service is available
        rospy.wait_for_service(request_service_name, timeout=3)
        request_service = rospy.ServiceProxy(request_service_name, ImageSrv)
    except Exception as e:
        rospy.logerr ("Exception:" + str(e))
        terminate()

    # Registering Publisher
    bounding_box_publisher = rospy.Publisher(image_topic_name, BoundingBox, queue_size=10)

    # Wait for initialization
    rospy.sleep(0.5)

def application():
    init()

    ##############################################################################
    ################### Example User Application #################################
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        try:
            image = get_image()
            rospy.loginfo("Image Received from AR Glass Successfully. Image size: [%d, %d]" %(image.shape[0], image.shape[1]))
            bounding_box_coordinates = process_image(image)
            send_bounding_box(bounding_box_coordinates)
        except Exception as e:
            rospy.logwarn ("Exception:" + str(e))

        rate.sleep()
    ##############################################################################


def terminate(*args):
    print ("Application Terminated")
    # image_publisher.unregister()
    # request_service.close()
    sys.exit()

if __name__ == '__main__':
    try:
        application()
    except rospy.ROSInterruptException:
        pass
    



    
