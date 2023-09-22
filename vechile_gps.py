#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW
from geojson import Feature,FeatureCollection,LineString
import json
import folium

data = []
data1=[]
data2=[]

def gps_callback(msg):
    latitude = msg.latitude
    longitude = msg.longitude
    data.append([longitude, latitude])

def gps_callback_mavros1(msg1):
    latitude1 = msg1.lat
    latitude1 = f"{str(latitude1)[:2]}.{str(latitude1)[2:]}"
    longitude1 = msg1.lon
    longitude1=f"{str(longitude1)[:2]}.{str(longitude1)[2:]}"
    data1.append([float(longitude1), float(latitude1)])

def gps_callback_mavros2(msg2):
    latitude2 = msg2.lat
    latitude2=f"{str(latitude2)[:2]}.{str(latitude2)[2:]}"
    longitude2 = msg2.lon
    longitude2=f"{str(longitude2)[:2]}.{str(longitude2)[2:]}"
    data2.append([float(longitude2),float(latitude2)])

def extract_gps_data():
    rospy.Subscriber('/vehicle/gps', NavSatFix, callback=gps_callback)
    rospy.Subscriber('/mavros/gpsstatus/gps1/raw', GPSRAW, callback=gps_callback_mavros1)
    rospy.Subscriber('/mavros/gpsstatus/gps2/raw', GPSRAW, callback=gps_callback_mavros2)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

    if data or data1 or data2:
        features = []

        # if data:
        #         feature = {
        #             "type": "Feature",
        #             "properties": {"color": "#ffcc90" },
        #             "geometry": {
        #                 "type": "LineString",
        #                 "coordinates": data,
        #             },
        #             "properties": {
        #                  "color":"33FF33"
        #                 #"time": rospy.get_time()
        #             }
        #         }
        #         features.append(feature)

        # if data1:
        #         feature1 = {
        #             "type": "Feature",
        #              "properties": {"color": "#ffcc90" },
        #             "geometry": {
        #                 "type": "LineString",
        #                 "coordinates": data1,
        #             },
        #             "properties": {
        #                 "color":"#339FFF"
        #                 #"time": rospy.get_time()
        #             }
        #         }
        #         features.append(feature1)

        # if data2:
        #         feature2 = {
        #             "type": "Feature",
        #              "properties": {"color": "#ffcc90" },
        #             "geometry": {
        #                 "type": "LineString",
        #                 "coordinates": data2,
        #             },
        #             "properties": {
        #                  "color":"3FF5733"
        #                 #"time": rospy.get_time()
        #             }
        #         }
        #         features.append(feature2)
    feature = Feature(geometry=LineString(data), properties={"strokeColor": "#f00","fillColor": "#0f0"})  # Color: Orange
    feature1 = Feature(geometry=LineString(data1), properties={"color": "#339FFF"})  # Color: Blue
    feature2 = Feature(geometry=LineString(data2), properties={"color": "#33FF33"})  # Color: Green
    # feature = Feature(folium.PolyLine(locations=data, color='red'))
    # feature1 = Feature(folium.PolyLine(locations=data, color='red'))
    # feature2 = Feature(folium.PolyLine(locations=data, color='red'))

    features = FeatureCollection([ feature,feature1, feature2])
    #features= folium.PolyLine(locations=features,color='red')
      

    with open('maindata.json', "w") as json_file:
        json.dump(features, json_file, indent=2)
        rospy.loginfo_once("JSON created successfully")
        rospy.signal_shutdown("Shutting down")

if __name__ == '__main__':
    try:
        rospy.init_node('gps_data_extractor')
        extract_gps_data()
        #rospy.signal_shutdown("Shutting down")
        

    except rospy.ROSInterruptException:
        pass
