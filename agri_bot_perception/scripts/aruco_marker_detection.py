#!/usr/bin/env python3

"""
Description: Obtaining TF of a ArUco marker using single sjcam camera
Algorithm: 
	Input: ROS topic for the RGB image
	Process: 
        - Subscribe to the image topic
		- Convert ROS format to OpenCV format
		- Detecting ArUco marker along with its ID
		- Applying perspective projection to calculate focal length
		- Extracting depth for each aurco marker
		- Broadcasting TF of each ArUco marker with naming convention as aruco1 for ID1, aruco2 for ID2 and so on.
	Output: TF of ArUco marker with respect to ebot_base
"""

import cv2 as cv
import numpy as np
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions

cv_image = None

def callback(data):
    # Initializing variables
    global cv_image
    focal_length = 476.70308
    center_x = 400.5
    center_y = 400.5
    aruco_dimension = 0.1
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        # load the dictionary that was used to generate the markers
        dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_1000)

        # initializing the detector parameters with default values
        parameters =  cv.aruco.DetectorParameters_create()

        # detect the markers in the frame
        corners, ids, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                
                pixel_width = topLeft[1] - bottomRight[1]

                # draw the ArUco marker ID on the frame
                cv.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                
                '''uncomment to view aruco ID and verify the working of the code'''
                #print("[INFO] ArUco marker ID: {}".format(markerID))

                # obtain depth for each ArUco marker
                distance = (focal_length*aruco_dimension)/pixel_width

                # transforming pixel coordinates to world coordinates
                world_x = (cX - center_x)/focal_length*distance
                world_y = (cY - center_y)/focal_length*distance
                world_z = distance

                # broadcasting TF for each aruco marker
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "sjcam_link"
                t.child_frame_id = "aruco"+str(markerID)

                # putting world coordinates coordinates as viewed for sjcam frame
                t.transform.translation.x = world_z
                t.transform.translation.y = -world_x
                t.transform.translation.z = world_y
                # not extracting any orientation thus orientation is (0, 0, 0)
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                br.sendTransform(t)

        '''uncoment to view the visual of detection'''
        cv.imshow("frame", frame)
        cv.waitKey(1)
    except CvBridgeError as e:
        print(e)


def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    # subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
    image_sub = rospy.Subscriber("/ebot/camera1/image_raw", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)