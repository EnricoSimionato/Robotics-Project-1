import os
import numpy as np
import time
import sys


minValue = 1000000.0
counter = 0
first = True
os.system("gnome-terminal -e 'bash -c \" catkin_make; roslaunch project1 project1.launch\"'")
for sum_lX_lY in np.arange(0.345,0.363,0.001):
	if first:
		parameter1 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator N " + str(43)
		parameter2 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator r " + str(0.078)
		parameter3 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator bag " + str(0)
		os.system("gnome-terminal -e 'bash -c \"" + parameter1 + "\"'")
		os.system("gnome-terminal -e 'bash -c \"" + parameter2 + "\"'")
		os.system("gnome-terminal -e 'bash -c \"" + parameter3 + "\"'")
		first = False
		time.sleep(7)
		os.system("gnome-terminal -e 'bash -c \"rostopic echo /calibration > distances.txt\"'")
	parameter1 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator sum_lX_lY " + str(sum_lX_lY)
	os.system("gnome-terminal -e 'bash -c \"" + parameter1 + "\"'")
	time.sleep(1)
	os.system("gnome-terminal -e 'bash -c \"cd bags/; rosbag play -u 104 bag2.bag\"'")		
	print(str(counter) + "\n")
	counter = counter + 1				
	time.sleep(104)


parameter1 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator sum_lX_lY " + str(sum_lX_lY)
os.system("gnome-terminal -e 'bash -c \"" + parameter1 + "\"'")
time.sleep(1)

f = open('distances.txt', 'r')
f.readline()
f.readline()
for sum_lX_lY in np.arange(0.345,0.363,0.001):
	result = f.readline()
	result1 = result[6:14]
	try:	
		res = float(result1)
		if(res < minValue):
			minValue = res
			param = sum_lX_lY
	except ValueError:
		print("error"+ str(sum_lX_lY))
	f.readline()
f.close
print(str(minValue) + " sum_lX_lY: " + str(param))
