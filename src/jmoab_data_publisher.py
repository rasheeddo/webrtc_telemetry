#!/usr/bin/env python

import rospy
import rospkg
import time
import argparse
import subprocess
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int8, UInt16, Bool
from geometry_msgs.msg import TwistWithCovarianceStamped
from jmoab_autopilot_ros.msg import GoalWaypoints
import json
import numpy as np
from datetime import datetime
import os

class DataPublisher(object):

	def __init__(self, NS, ID, PUSH, PORT):

		rospy.init_node('data_publisher_node', anonymous=True)

		rospack = rospkg.RosPack()
		webrtc_telem_ros_path = rospack.get_path("webrtc_telemetry")

		if NS is None:
			telem_file_name = "telemetry.txt"
		else:
			telem_file_name = NS + "_telemetry.txt"

		self.file_path = os.path.join(webrtc_telem_ros_path, "tmp", telem_file_name)

		if PUSH:
			self.push_data = True
			self.console_port = PORT
		else:
			self.push_data = False

		self.telem_dict = {
							"telemetry": {
							"pos": {
									"lat": 0.0, "lon": 0.0, "alt": 0.0},
							"att": {
									"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
							"dist": {"travelled": 0.0, "toHome": 0.0, "toNextWp": 0.0},
							"speed": {"air": 0., "ground": 0.0},
							"nav": {"nextWp": 0, "eta":[]},
							"mode": "MAN",
							"gps": "",
							"batt": {"volt": 0.0, "current": 0.0}
							},
							"id": ID	
						}

		self.lat = 0.0
		self.lon = 0.0
		self.prev_lat = 0.0
		self.prev_lon = 0.0
		self.cart_mode = "MAN"
		self.travelled = 0.0
		self.distToNextWp = 0.0
		self.abs_vel = 0.0
		self.next_wp = 0
		self.got_reply_mission = False

		self.total_wps = 0
		self.mission_completed = False
		self.prev_wp = 0

		if NS is None:
			gps_topic = "/ublox/fix"
			compass_topic = "/jmoab_compass"
			vel_topic = "/ublox/fix_velocity"
			mode_topic = "/atcart_mode"
			adc_topic = "/jmoab_adc"
			arrived_wp_topic = "/arrived_waypoints"
			request_wp_topic = "/request_store_waypoints"
			mission_topic = "/reply_goal_waypoints"
			mis_com_topic = "/mission_completed"
		else:
			if NS.startswith("/"):
				gps_topic = NS + "/ublox/fix"
				compass_topic = NS + "/jmoab_compass"
				vel_topic = NS + "/ublox/fix_velocity"
				mode_topic = NS + "/atcart_mode"
				adc_topic = NS + "/jmoab_adc"
				arrived_wp_topic = NS + "/arrived_waypoints"
				request_wp_topic = NS + "/request_store_waypoints"
				mission_topic = NS + "/reply_goal_waypoints"
				mis_com_topic = NS + "/mission_completed"
			else:
				gps_topic = "/" + NS + "/ublox/fix"
				compass_topic = "/" + NS + "/jmoab_compass"
				vel_topic = "/" + NS + "/ublox/fix_velocity"
				mode_topic = "/" + NS + "/atcart_mode"
				adc_topic = "/" + NS + "/jmoab_adc"
				arrived_wp_topic = "/" + NS + "/arrived_waypoints"
				request_wp_topic = "/" + NS + "/request_store_waypoints"
				mission_topic = "/" + NS + "/reply_goal_waypoints"
				mis_com_topic = "/" + NS + "/mission_completed"

		#########################
		## Telemetry subsciber ##
		#########################
		rospy.Subscriber(compass_topic, Float32MultiArray, self.compass_callback)
		rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback)
		rospy.Subscriber(vel_topic, TwistWithCovarianceStamped, self.vel_callback)
		rospy.Subscriber(mode_topic, Int8, self.atcart_mode_callback)
		rospy.Subscriber(adc_topic, Float32MultiArray, self.adc_callback)
		########################
		## Mission subscriber ##
		########################
		rospy.Subscriber(arrived_wp_topic, UInt16, self.arrived_wp_callback)
		rospy.Subscriber(mission_topic, GoalWaypoints, self.robot_mission_callback)
		rospy.Subscriber(mis_com_topic, Bool, self.mission_completed_callback)
		self.request_wp_pub = rospy.Publisher(request_wp_topic, Bool, queue_size=10)
		self.request_wp_msg = Bool()


		self.loop()
		rospy.spin()

	def get_distance(self, lat1, lon1, lat2, lon2):

		R = 6371.0*1000.0
		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		a = np.sin(dLat/2.0)*np.sin(dLat/2.0) + np.cos(lat_start)*np.cos(lat_end)*np.sin(dLon/2.0)*np.sin(dLon/2.0)
		c = 2.0*np.arctan2(np.sqrt(a),np.sqrt(1-a))

		d = c*R

		return d

	def compass_callback(self, msg):
		self.telem_dict["telemetry"]["att"]["roll"] = round(msg.data[0],2)
		self.telem_dict["telemetry"]["att"]["pitch"] = round(msg.data[1],2)
		self.telem_dict["telemetry"]["att"]["yaw"] = round(msg.data[2],2)

	def gps_callback(self, msg):

		self.prev_lat = self.lat
		self.prev_lon = self.lon

		self.lat = msg.latitude
		self.lon = msg.longitude
		self.telem_dict["telemetry"]["pos"]["lat"] = round(self.lat,7) 
		self.telem_dict["telemetry"]["pos"]["lon"] = round(self.lon,7)

		if msg.status.status == -1:
			self.telem_dict["telemetry"]["gps"]= ""
		elif msg.status.status == 0:
			self.telem_dict["telemetry"]["gps"]= "3D"
		elif msg.status.status == 1:
			self.telem_dict["telemetry"]["gps"]= "DGPS"
		elif msg.status.status == 2:
			self.telem_dict["telemetry"]["gps"]= "RTKFXD"

		self.cart_mode = self.telem_dict["telemetry"]["mode"]

		if (self.lat != 0.0) and (self.lon != 0.0) and (self.prev_lat != 0.0) and (self.prev_lon != 0.0) and (self.cart_mode == "AUTO"):
			self.travelled += self.get_distance(self.prev_lat, self.prev_lon, self.lat, self.lon)
			self.telem_dict["telemetry"]["dist"]["travelled"] = round(self.travelled, 2)

			if self.got_reply_mission:
				self.telem_dict["telemetry"]["dist"]["toNextWp"] = round(self.get_distance(self.lat, self.lon, self.lat_target_list[self.next_wp-1], self.lon_target_list[self.next_wp-1]),2)

	def vel_callback(self, msg):
		vel_x = msg.twist.twist.linear.x
		vel_y = msg.twist.twist.linear.y
		self.abs_vel = np.sqrt(vel_x**2 + vel_y**2)
		self.telem_dict["telemetry"]["speed"]["air"] = round(self.abs_vel,2)
		self.telem_dict["telemetry"]["speed"]["ground"] = round(self.abs_vel,2)

	def atcart_mode_callback(self, msg):
		if (msg.data == 0) or (msg.data == 1):
			mode = "MAN"
			self.cart_mode = "MAN"
		elif (msg.data == 2):
			mode = "AUTO"
			self.cart_mode = "AUTO"

		self.telem_dict["telemetry"]["mode"] = mode

	def adc_callback(self, msg):
		self.telem_dict["telemetry"]["batt"]["volt"] = round(msg.data[0],2)

	def arrived_wp_callback(self, msg):

		self.prev_wp = self.next_wp

		if msg.data < self.total_wps:
			self.next_wp = msg.data + 1
		elif msg.data == self.total_wps:
			self.next_wp = 1
		else:
			self.next_wp = 0

		self.telem_dict["telemetry"]["nav"]["nextWp"] = self.next_wp

	def robot_mission_callback(self, msg):
		self.lat_target_list = msg.lat.data
		self.lon_target_list = msg.lon.data
		self.speed_target_list = msg.speed.data
		self.delay_target_list = msg.delay.data
		self.relay_target_list = msg.relay.data

		self.total_wps = len(self.lat_target_list)
		self.got_reply_mission = True

		print("Got mission from robot")

	def mission_completed_callback(self, msg):

		self.mission_completed = msg.data

	def calculate_ETA(self):

		# global target_lat_list, target_lon_list, total_points
		# global groundspeed, wp_speed

		ETA_list = []
		linuxTime_list = []

		## nextwp in Ardupilot never be 0, always start from 1
		for i in range(self.total_wps):

			################################
			## Points that already passed ##
			################################
			if i < (self.next_wp-1):

				ETA_passed = "Passed"
				ETA_list.append(ETA_passed)
				linuxTime_list.append(time.time())

			#############################
			## Current point to nextwp ##
			#############################
			elif i == (self.next_wp-1):
				
				dist_to_next = self.get_distance(self.lat, self.lon, self.lat_target_list[self.next_wp-1], self.lon_target_list[self.next_wp-1])

				if self.abs_vel == 0.0:
					vel = self.speed_target_list[self.next_wp-1]
				else:
					vel = self.abs_vel

				elaspedTime_to_next_in_sec = dist_to_next/vel
				linuxTime_to_next = time.time() + elaspedTime_to_next_in_sec
				human_time_to_next = datetime.fromtimestamp(linuxTime_to_next)
				ETA_next = human_time_to_next.strftime("%H:%M:%S")
				ETA_list.append(ETA_next)
				linuxTime_list.append(linuxTime_to_next)

			##########################
			## Points in the future ##
			##########################
			elif i > (self.next_wp-1):
				dist_to_next = self.get_distance(self.lat_target_list[i-1], self.lon_target_list[i-1], self.lat_target_list[i], self.lon_target_list[i])
				vel = self.speed_target_list[self.next_wp-1]

				elaspedTime_to_next_in_sec = dist_to_next/vel
				linuxTime_to_future = linuxTime_list[i-1] + elaspedTime_to_next_in_sec
				human_time_to_future = datetime.fromtimestamp(linuxTime_to_future)
				ETA_future = human_time_to_future.strftime("%H:%M:%S")
				ETA_list.append(ETA_future)
				linuxTime_list.append(linuxTime_to_future)

			if (self.abs_vel < 0.1) and (self.next_wp == self.total_wps):
				ETA_list = ["Passed"]*self.total_wps

		return ETA_list


	def loop(self):

		rate = rospy.Rate(20)

		while not rospy.is_shutdown():

			if not self.got_reply_mission:
				self.request_wp_msg.data = True
				self.request_wp_pub.publish(self.request_wp_msg)

			if not self.mission_completed:
				if (self.cart_mode == "AUTO") and (self.next_wp != 0):
					ETA_list = self.calculate_ETA()
					print(ETA_list)
				else:
					ETA_list = []
			else:
				ETA_list = []

			self.telem_dict["telemetry"]["nav"]["eta"] = ETA_list

			json_data = json.dumps(self.telem_dict)
			file = open(self.file_path, "w+")
			file.write(json_data)

			if self.push_data:
				cmd1 = 'echo $(cat {:s}) > {:s}'.format(self.file_path, self.console_port)
				#subprocess.run(cmd1, shell=True, check=True)	# for python3
				subprocess.call(cmd1, shell=True)				# for python2
			

			rate.sleep()



if __name__ == "__main__":

	parser = argparse.ArgumentParser(description='webrtc telemetry data publisher node')
	parser.add_argument('--ns',
					help="This is a namespace of the robot")
	parser.add_argument('--id',
						help="ID of vehicle")
	parser.add_argument('--push',
						help="0: save data to file but not pushing to /dev/pts/* yet, 1: push telemetry data to /dev/pts/* port")
	parser.add_argument('--console_port',
						help="This is a second port generated by 1st_socat.sh")

	args = parser.parse_args(rospy.myargv()[1:])
	ns = args.ns
	push = args.push
	console_port = args.console_port

	if ns is not None:
		print("Use namespace as {:}".format(ns))
	else:
		print("No namespace, using default")

	if args.id is None:
		print("Use id 1 as default")
		_id = 1
	else:
		_id = int(args.id)

	if (push is None) or (push == 0):
		print("Store telemetry data to tmp/ as defaul, and no push")
		console_port = None
		PUSH = False
	else:
		if console_port is None:
			print("Please specify console_port")
			quit()
		else:
			print("Push data to console_port {:}".format(console_port))
			PUSH = True

	data_publisher = DataPublisher(ns,_id, PUSH, console_port)
