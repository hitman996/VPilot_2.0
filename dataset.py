# coding=UTF-8
from deepgtav.messages import Start, Stop, Config, Dataset, frame2numpy, Scenario
from deepgtav.client import Client
import argparse
import time
import cv2
import matplotlib.pyplot as plt
import os
import numpy as np

FOLLOWING_TRAFFICS = 0x1 
YIELD_TO_CROSSING_PEDS = 0x2 
DRIVE_AROUND_VEHICLES = 0x4 
DRIVE_AROUND_EMPTY_VEHICLES = 0x8 
ORI_WIDTH = 1248
ORI_HEIGHT = 374
DRIVE_AROUND_PEDS = 0x10 
DRIVE_AROUND_OBJECTS = 0x20 
UNKOWN_1 = 0x40 
STOP_AT_TRAFFIC_LIGHTS = 0x80 

USE_BLINKERS = 0x100 
ALLOW_GOING_WRONG_WAY = 0x200  	
GO_IN_REVERSE_GEAR = 0x400  	# backwards
UNKOWN_2 = 0x800 			

UNKOWN_3 = 0x1000 	
UNKOWN_4 = 0x2000 
UNKOWN_5 = 0x4000 
UNKOWN_6 = 0x8000 

UNKOWN_7 = 0x10000 
UNKOWN_8 = 0x20000 
TAKE_SHORTEST_PATH = 0x40000  	# Removes most pathing limits  the driver even goes on dirtroads. 没有它也能超车
ALLOW_LANE_CHANEG_OVERTAKE = 0x80000 

UNKOWN_9 = 0x100000 
UNKOWN_10 = 0x200000 
IGNORE_ROADS = 0x400000  		# Uses local pathing  only works within 200~meters around the player
UNKOWN_11 = 0x800000 

IGNORE_ALL_PATHING = 0x1000000  # Goes straight to destination
UNKOWN_12 = 0x2000000 			
UNKOWN_13 = 0x4000000 			# maybe avoid too close laterally
UNKOWN_14 = 0x8000000  			

UNKOWN_15 = 0x10000000 
AVOID_HIGHWAYS_WHEN_POSSIBLE = 0x20000000  # will use the highway if there is no other way to get to the destination
UNKOWN_16 = 0x40000000
UNKOWN_17 = 0x80000000
    
DEFAULT = DRIVE_AROUND_EMPTY_VEHICLES | DRIVE_AROUND_OBJECTS  | USE_BLINKERS | STOP_AT_TRAFFIC_LIGHTS
DUAL_STOP = FOLLOWING_TRAFFICS | YIELD_TO_CROSSING_PEDS
DUAL_AVOID = DRIVE_AROUND_VEHICLES | DRIVE_AROUND_PEDS
LANE_SELECT_1 = 0
LANE_SELECT_2 = ALLOW_LANE_CHANEG_OVERTAKE
LANE_SELECT_3 = ALLOW_LANE_CHANEG_OVERTAKE | TAKE_SHORTEST_PATH | ALLOW_GOING_WRONG_WAY

STRICT_1 = DEFAULT | DUAL_STOP | LANE_SELECT_1 
STRICT_2 = DEFAULT | DUAL_STOP | LANE_SELECT_2
LOOSE_1 = DEFAULT | DUAL_STOP | DUAL_AVOID | LANE_SELECT_2 
LOOSE_2 = DEFAULT | DUAL_STOP | DUAL_AVOID | LANE_SELECT_3
LOOSE_3 = DEFAULT | DUAL_AVOID | LANE_SELECT_3
AVOID_PED = DEFAULT | DUAL_AVOID | LANE_SELECT_3


# Stores a pickled dataset file with data coming from DeepGTAV
if __name__ == '__main__':
	parser = argparse.ArgumentParser(description=None)
	parser.add_argument('-l', '--host', default='localhost', help='The IP where DeepGTAV is running')
	parser.add_argument('-p', '--port', default=8000, help='The port where DeepGTAV is running')
	parser.add_argument('-d', '--dataset_path', default='D:\\Git_repositories\\VPilot-master\\KITTI', help='Place to store the dataset')
	args = parser.parse_args()

	# Creates a new connection to DeepGTAV using the specified ip and port 
	client = Client(ip=args.host, port=args.port, datasetPath=args.dataset_path, compressionLevel=9 , divideByTrip=True) 
	
	dataset = Dataset(rate=20 ,frame=[1920,1080],  throttle=True, brake=True, steering=True, speed=True, acceleration=True, yaw=True, yawRate=True, isCollide=True, location=True,
		drivingModeMsg=False)# lidar=[3, True, 100.0, 1000, 60.0, 300.0, 20, 85.0, 115.0], , vehicles=True, peds=True
	# Automatic driving scenario
	scenario = Scenario(weather='EXTRASUNNY',vehicle='blista', time=[12,0], drivingMode=[-2, 3, 25.0, 1.0, 1.0], 
					route=[	\
			-1989.000000, -468.250000, 10.562500, 
			1171.351563, -1925.791748, 36.220097
			]) 
	

	client.sendMessage(Start(scenario=scenario,dataset=dataset)) # Start request
	count = 0
	tripNum = 0
	path = 'D:\\Git_repositories\\VPilot-master\\KITTI\\'
	old_location = [0, 0, 0]
	while True: # Main loop
		try:
			count +=1
			# name = os.path.join(str(count), '.png')
			# Message recieved as a Python dictionary
			message = client.recvMessage()
			image = frame2numpy(message['frame'],  (1920,1080))
			image = image[int((1080-ORI_HEIGHT) / 2) : int(1080 - ((1080-ORI_HEIGHT) / 2)),
						  int((1920-ORI_WIDTH) / 2): int(1920-((1920-ORI_WIDTH) / 2))]
			# print(image.shape)

			dest = os.path.join(path, str(str(count) + '.png'))
			status = cv2.imwrite(dest,image)
			if status:
				print ('File saved')
			else:
				print('File was not saved.')
			# cv2.imshow('img', image)
			# cv2.waitKey(-1)
			if (count % 500) == 0:
				print(count)

			if 'drivingMode' in message:
				if message['drivingMode'][0] == 0:
					print('trip' + str(tripNum))
					tripNum += 1

			count += 1

		except KeyboardInterrupt:
			i = input('Paused. Press p to continue and q to exit... ')
			if i == 'p':
				continue
			elif i == 'q':
				break
			
	# DeepGTAV stop message
	client.sendMessage(Stop())
	client.close() 


"""
Positions:
	location=[-1484.750000, 2039.500000, 62.406250]							# village
	location=[-1590.750000, -162.250000, 54.562500] 						# city
	location=[-2576.500000, 3293.000000, 12.375000]							# tunnel 

	# village to tunnel
	route=[-1484.750000, 2039.500000, 62.406250, 
			-2576.500000, 3293.000000, 12.375000]

	# highway1-village
	location=[-2203.527344, -337.456299, 13.119763]							# south western coast road starting point
	location=[-2816.843994, 2193.854980, 29.460247]							# western coast road middle point, befor the bridge to the tunnle
	location [-722.478088, 5522.071289, 36.470577]
	location=[-437.375763, 5917.432617, 32.426182]
	route=[-2203.527344, -337.456299, 13.119763, 
			-2816.843994, 2193.854980, 29.460247]
	route=[-437.375763, 5917.432617, 32.426182, 	
			-722.478088, 5522.071289, 36.470577, 
			-2816.843994, 2193.854980, 29.460247, 
			-2203.527344, -337.456299, 13.119763]

	# highway2-city
	location = [-1989.000000, -468.250000, 10.562500]						# western coast road
	location = [-1037.957886, -607.505615, 18.152155]						# middle pos
	location = [438.339630, -523.561157, 35.797802]							# middle pos
	location = [1000.059448, -906.553833, 30.398233]						# middle pos
	location = [1053.914917, -1533.013306, 27.520031]						# middle pos
	location = [1171.351563, -1925.791748, 36.220097]						# middle eastern city, industrial zone
	route=[	-1989.000000, -468.250000, 10.562500, 
			-1037.957886, -607.505615, 18.152155,
			438.339630, -523.561157, 35.797802,
			1000.059448, -906.553833, 30.398233,
			1171.351563, -1925.791748, 36.220097]

	# city
	location=[-1900.778442, -203.396835, 36.310143]							# western city
	location=[689.279053, 26.910444, 83.943283]								# eastern city
	route=[-1900.778442, -203.396835, 36.310143, 
			689.279053, 26.910444, 83.943283]


Datasets:
	lidar args, average samples:
		args0: int, lidar state flag:
			LIDAR_NOT_INIT_YET,
			LIDAR_INIT_AS_2D,
			LIDAR_INIT_AS_3D_CONE,
			LIDAR_INIT_AS_3D_SCALED_CONE,
			LIDAR_INIT_AS_3D_SPACIALCIRCLE,
			LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE
		args1: bool, visualize the laser dots
		args2: float, lidar laser max range
		args3: int, horizontal sample number
		args4: float, horizontal angle left limit, degree
		args5: float, horizontal angle right limit, degree
		args6: int, vertical sample number
		args7: float, vertical upper limit, degree
		args8: float, vertical under limit, degree

		example: 
			lidar=[1, False, 100.0, 1080, 90.0, 270.0]	# 2D lidar
			lidar=[2, False, 100.0, 180, 90.0, 270.0, 15, 85.0, 130.0]	# 3D lidar

	lidar args, scaled samples, 3D lidar only:
		args0: int, lidar state flag, 3-LIDAR_INIT_AS_3D_SCALED, LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE
		args1: bool, visualize the laser dots
		args2: float, lidar laser max range
		args3: int, total sample number
		args4: float, horizontal angle left limit, degree
		args5: float, horizontal angle right limit, degree
		args6: int, vertical sample number
		args7: float, vertical upper limit, degree
		args8: float, vertical under limit, degree

		example:
			lidar=[3, False, 100.0, 1200, 60.0, 300.0, 20, 85.0, 115.0]


Scenario args:
	surroundDrivingMode args:
		args0: int params flag:
			args0 == -2 use default GTAV AI driver, args1~3 are useless
			args0 == -1 use preinstall driving style as the surrounding driving mode, args2~3 are useless
			args0 >=  0 use manual setting driving style. And this args is the driving style int
		args1: an args depends on args0
			if args0 == -2, it's useless
			if args0 == -1, it's the preinstall driving style index
			if args0 >=  0, it's the desired speed
		args2: only activated if args0 >= 0
			aggressiveness
		args3: only activated if args0 >= 0
			ability
		example:
			use default GTAV AI driver: 
				[-2]
			use the 2rd(begins from 0) preinstall driving style: 
				[-1, 1]
			use manual driving style, set desired speed=30.0, aggressiveness=0.0, ability=1.0: 
				[RUSHED, 30.0, 0.0, 1.0]

	drivingMode args:
		args0: int, ego driving style index
			args0 == -2	Manual driving:	just drive, args1~args4 are useless
			args0 == -1	Auto driving:	auto selecting preinstall driving styles, desired speed. manual setting route mode by turns when the vehicle reach the destination. Args2~args4 are useless
			args0 >=  0	Auto driving:	manual setting  driving styles, desired speed, driving aggressiveness, driving ability and route mode
		args1: if args0 >= -1, int, ego route mode, WANDERING==0, TO_COORD_ONE_WAY_TRIP==1, TO_COORD_CIRCLE_TRIP==2, TO_COORD_ONE_WAY_TRIP_CIRCLE==3
		args2: if args0 >=  0, float, ego desired speed
		args3: if args0 >=  0, float, ego driving aggressiveness of driver
		args4: if args0 >=  0, float, ego driving ability of driver
		example: 
			use manual driving:
				[-2] or None
			use preinstall driving style:
				[-1]
			use manual driving style, set driving style int=3, TO_COORD_ONE_WAY_TRIP_CIRCLE, desired speed=30.0, aggressiveness=1.0, ability=1.0:
				[3, 2, 30, 1.0, 1.0]
		
	route args:
		args0~args2: float, start position XYZ
		args3~args5: float, destination or middle position
		...
		argsn~argsn+2: float, destination
		example: 
			route=[-1590.750000, -162.250000, 54.562500, -1592.500000, -197.500000, 54.281250]
			test loop: [-1590.750000, -162.250000, 54.562500] to [-1592.500000, -197.500000, 54.281250]

"""
