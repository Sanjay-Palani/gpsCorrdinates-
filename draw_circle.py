#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW
import json
import os

# Global flag to ensure data is written only once
file_written = False

# Variable declaration
data = []
data1 = []
data2 = []
json_file_path = 'maindata.json'

# Callback function for topic vehicle/gps
def gps_callback(msg):
    global data
    rospy.loginfo_once("vehicle info call back")
    latitude = msg.latitude
    longitude = msg.longitude
    data.append([longitude, latitude])

# Callback function for topic mavros/gpsstatus/gps1/raw
def gps_callback_mavros1(msg1):
    global data1
    rospy.loginfo_once("gps1 info call back")
    latitude1 = msg1.lat
    longitude1 = msg1.lon
    data1.append([longitude1, latitude1])

# Callback function for topic mavros/gpsstatus/gps2/raw
def gps_callback_mavros2(msg2):
    global data2
    rospy.loginfo_once("gps2 info call back")
    latitude2 = msg2.lat
    longitude2 = msg2.lon
    data2.append([longitude2, latitude2])

def extract_gps_data():
    global file_written  # Use the global flag

    rospy.Subscriber('/vehicle/gps', NavSatFix, callback=gps_callback)
    rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, callback=gps_callback_mavros1)
    rospy.Subscriber('/mavros/gpsstatus/gps2/raw', GPSRAW, callback=gps_callback_mavros2)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

        if data or data1 or data2:
            features = []

            if data:
                feature = {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": data,
                    },
                    "properties": {
                        "time": rospy.get_time()
                    }
                }
                features.append(feature)

            if data1:
                feature1 = {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": data1,
                    },
                    "properties": {
                        "time": rospy.get_time()
                    }
                }
                features.append(feature1)

            if data2:
                feature2 = {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": data2,
                    },
                    "properties": {
                        "time": rospy.get_time()
                    }
                }
                features.append(feature2)

            if os.path.exists(json_file_path):
                # Load existing data from JSON
                with open(json_file_path, 'r') as json_file:
                    existing_data = json.load(json_file)
                features.extend(existing_data)

            with open(json_file_path, "w") as json_file:
                json.dump(features, json_file, indent=2)
                rospy.loginfo_once("Data appended to maindata.json")

                file_written = True  # Set flag to True after writing data

if __name__ == '__main__':
    try:
        rospy.init_node('gps_data_extractor')
        extract_gps_data()

        # Add a while loop to prevent the program from exiting immediately
        while not file_written and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.signal_shutdown("shutdowning")

    except rospy.ROSInterruptException:
        pass
