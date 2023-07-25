#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tensorflow as tf
import h5py
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from keras.models import load_model
from random import uniform
import sys

class drive_auto:
    #IMAGE_SCALE = 4
    model = None
    autodrive_publisher = None
    graph = None
    terminate = False

    def __init__(self):
        #self.model = load_model("1weights.432-0.94&3.54.h5")
        #self.model = load_model("2weights.967-0.61&4.62.h5")
        #self.model = load_model("3weights.019-1.95&2.06.h5")
        #self.model = load_model("final3.h5")
        self.model = load_model("model_home.h5")
        self.graph = tf.get_default_graph()
        self.terminate = False

    def linear_unbin1(self, arr):
        #a=arr
        b = np.argmax(arr)
        a = b *(2/14) - 1.0
        return a

    def linear_unbin(self, value_to_unbin):
        unbinned_value = value_to_unbin * (2.0 / 14.0) - 1

        return unbinned_value
    
    def unbin_matrix(self, matrix_to_unbin):
        unbinned_matrix=[]
        
        for value_to_unbin in matrix_to_unbin:
            unbinned_value = np.argmax(value_to_unbin)
            unbinned_value = self.linear_unbin(unbinned_value)
            unbinned_matrix.append(unbinned_value)

        return np.array(unbinned_matrix)

    def feedforward(self, message):
        array=ByteMultiArray()
        array.data.clear()
        np_arr = np.fromstring(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image = cv2.resize(image, (640,480), interpolation=cv2.INTER_AREA)
        image = cv2.resize(image, (160,120), interpolation=cv2.INTER_AREA)
        w,h=image.shape[:2]
        center=(h/2,w/2)
        M=cv2.getRotationMatrix2D(center,180,1.0)
        image=cv2.warpAffine(image,M,(h,w))
        #image = cv2.resize(image, (160,120), interpolation=cv2.INTER_AREA)
        #image = image[120:,:]
        image = np.asarray(image)
        images = []
        images.append(image)
        with self.graph.as_default():
            prediction = self.model.predict(np.array(images), batch_size=1)
            #steering_angle = prediction[0][0]
            steering_angle = self.linear_unbin1(prediction)
            throttle = 70
            #throttle = rospy.get_param("/THROTTLE_AUTO_DRIVE")
            #throttle = prediction[1][0][0]
            array.data.append(int(steering_angle))
            array.data.append(int(throttle))
            command = str(steering_angle) + ":" + str(throttle)
            if self.terminate == False:
                rospy.loginfo("Autopilot command: " + command)
                self.autodrive_publisher.publish(array)

    def shutdown(self):
        self.terminate = True
        rospy.loginfo("Setting steering angle and throttle to 0")
        self.autodrive_publisher.publish("0:0")
        self.autodrive_publisher.publish("0:0")
        self.autodrive_publisher.publish("0:0")


def listener(Node):

    rospy.init_node("self_driving_node", log_level=rospy.DEBUG)
    #imageProcessingNode.autodrive_publisher = rospy.Publisher("autodrive", String, queue_size=1)
    Node.autodrive_publisher = rospy.Publisher("autodrive", ByteMultiArray, queue_size=1)
    #initialising
    rospy.Subscriber("/lanedetectcam/usb_cam/image_raw/compressed", CompressedImage, Node.feedforward, queue_size=1)
    #rospy.Subscriber("/lanedetectcam/usb_cam/image_raw/", Image, Node.feedforward, queue_size=1)
    rospy.on_shutdown(Node.shutdown)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    drive_auto_Node = drive_auto()
    listener(drive_auto_Node)