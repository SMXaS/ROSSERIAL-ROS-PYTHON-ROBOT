#!/usr/bin/env python 

# Always import python interpreter when working with ROS

"""Importing:
* ROS,
* Type of the message,
* File opening OS,
* Numpy Library,
* Tensorflow and Keras, model functions from it, 
* Tensorflow FrontEnd,
* Keras BackEnd.
"""
import rospy
from std_msgs.msg import String
import os
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import load_img, img_to_array

"""Function Prepare Image:
* As a parameter takes path of the data,
* Loads the image from the path and sets up the parameter for the image,
* Prepares the image into the numpy array and returns it.
"""
def prepare_image(data_path):
    image = load_img(data_path, target_size=(30, 30))
    image = img_to_array(image)
    image = np.expand_dims(image, axis=0)
    return image

"""Prediction of the model function:
* Loads the prepared model from the data path, 
* All the images are assigned to the labels,
* The image that is needed, goes through the parameters,
* The image gets resized and labeled,
* Predictions returning the Image label text
* The picture can be displayed with matplotlib MATLAB function.
"""

def predict_model(data_path, model):
    labels = ['20 km/h', '30 km/h', '50 km/h', '60 km/h', '70 km/h', '80 km/h', '80 km/h end', '100 km/h', 
             '120 km/h', 'No overtaking','No overtaking for tracks', 'Crossroad with secondary way', 
             'Main road', 'Give way', 'Stop', 'Road up', 'Road up for track', 'Brock','Other dangerous',
             'Turn left', 'Turn right', 'Winding road', 'Hollow road', 'Slippery road', 'Narrowing road',
             'Roadwork', 'Traffic light', 'Pedestrian', 'Children', 'Bike', 'Snow', 'Deer', 'End of the limits',
             'Only right', 'Only left', 'Only straight', 'Only straight and right', 'Only straight and left',
             'Take right', 'Take left', 'Circle crossroad', 'End of overtaking limit', 'End of overtaking limit for track']
    
    image = prepare_image(data_path)
    prediction_data = np.argmax(model.predict(image), axis=1)

    prediction = labels[prediction_data[0]]
    print(prediction)
    return prediction

"""Operating system OS:
* Helps to open the file and use GPU or CPU
"""
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

"""The Path of the saved model:
* Helps, to save the trained model so there is no need to retrain it every time.
"""
MODEL_PATH = '/home/wu/catkin_ws/src/robot_arduino/src/data/scripts/model-1.h5'

"""Model Loading"""
model = load_model(MODEL_PATH)

"""Main function:
* Sets the Path of the predicted image,
* Can Be selected any picture from the /data folder,
* Sign is assigned as predicted model which opens the prepared model,
* Returns the predicted image.
* Uncomment the path to make it recognize other sign,
* Has a minor problem, it may recognize the signs as a totally different one, so check it twice +
* + upload many signs to test, there is no issues with the code, just images+model low prediction.
"""
def main():
    #file_path = f'/home/wu/catkin_ws/src/robot_arduino/src/data/forward.png'
    #file_path = f'/home/wu/catkin_ws/src/robot_arduino/src/data/stop.png'
    #file_path = f'/home/wu/catkin_ws/src/robot_arduino/src/data/turn_left.jpeg'
    file_path = f'/home/wu/catkin_ws/src/robot_arduino/src/data/right.png'
    sign = predict_model(file_path, model)
    return sign

"""Signs Publisher function:
* Stores the empty string called msg,
* Assigns the new_sign as a main function ( in Main we have the prepared Image),
* Contains if-else statements,
* If- Else working like a debug, 
* IF the imported image is == recognized as this image send this message,
* IF the imported image is == not recognized, the Robot drives forward or error occurs,
* Contains a message like in Arduino, Type of the message and the size of how many messages,
* Also, it goes through ROS initialization of the Node, publishes the topic,
* Topic has to be the same as the def signs_pub variable,
* It sends anonymous message,
* Rate is for the how fast the message you want to be executed,
* Moreover, it goes through while loop where if ROS not shutdown the program +
* + still works and allows the communication between publisher and subscriber,
* In the end, while loop also contains the same concept of signs pub IF-Else,
* If the image is recognised it is publishing it as a message and lets to Subscribe it.
"""

def signs_pub():
    msg = ''
    new_sign = main()
    if new_sign == 'Stop':
        signs_publisher = rospy.Publisher('motor_stop', String, queue_size=10)
    if new_sign == 'Only right':
        signs_publisher = rospy.Publisher('motor_right', String, queue_size=10)
    if new_sign == 'Only left':
        signs_publisher = rospy.Publisher('motor_left', String, queue_size=10) 
    if new_sign == 'Only straight': 
        signs_publisher = rospy.Publisher('motor_forward', String, queue_size=10)

    rospy.init_node('signs_pub', anonymous=True)
    rate = rospy.Rate(10)  # 1hz
    while not rospy.is_shutdown():
        if new_sign == 'Stop':
            signs_publisher.publish(msg)
        if new_sign == 'Only right':
            signs_publisher.publish(msg)
        if new_sign == 'Only left':
            signs_publisher.publish(msg)
        if new_sign == 'Only straight': 
            signs_publisher.publish(msg)
        rospy.loginfo(new_sign)
        rate.sleep()

"""Execution of the main function,
* Recommended to have in all programs in order to not run the whole program, but part of it,
* If the signs_pub function is working pass, if not throw an ROS error. 
"""
if __name__ == '__main__':
    try:
        signs_pub()
    except rospy.ROSInterruptException:
        pass