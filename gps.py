#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW
from geojson import Feature,FeatureCollection,LineString
import json
data=[]
data1=[]
data2=[]

def vehicle_data(msg):
    latitude = msg.latitude
    longitude = msg.longitude
    data.append([longitude,latitude])
def gps1_data(msg1):
    latitude1=msg1.lat
    latitude1=f"{str(latitude1)[:2]}.{str(latitude1)[2:]}"
    longitude1=msg1.lon
    longitude1=f"{str(longitude1)[:2]}.{str(longitude1)[2:]}"
    data1.append([float(longitude1),float(latitude1)])
def gps2_data(msg2):
    latitude2=msg2.lat
    latitude2=f"{str(latitude2)[:2]}.{str(latitude2)[2:]}"
    longitude2=msg2.lon

    longitude2=f"{str(longitude2)[:2]}.{str(longitude2)[2:]}"
    data2.append([float(longitude2),float(latitude2)])
    

def extract_data():
    rospy.Subscriber("/vehicle/gps",NavSatFix,callback=vehicle_data)
    rospy.Subscriber("/mavros/gpsstatus/gps1/raw",GPSRAW,callback=gps1_data)
    rospy.Subscriber("/mavros/gpsstatus/gps2/raw",GPSRAW,callback=gps2_data)
    
    
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    
        features=[]

    feature=Feature(geometry=LineString(data))
    feature1=Feature(geometry=LineString(data1))
    feature2=Feature(geometry=LineString(data2))
    features=FeatureCollection([feature,feature1,feature2])
    
    with open('maindata5.json',"w") as json_file:
        json.dump(features,json_file,indent=2)
        rospy.loginfo_once("json file is created")
        rospy.signal_shutdown("shutdowning")

if __name__=="__main__":
    try:
        rospy.init_node("gps_data")
        extract_data()
    except rospy.ROSInterruptException:
        pass