import os
import numpy as np
import time
import sys

minValue = 1000000.0
counter = 0
first = True
os.system("gnome-terminal -e 'bash -c \" catkin_make; roslaunch project1 project1.launch\"'")
for r in np.arange(0.073,0.08,0.001):
	for n in np.arange(42,47,1):
		if first:
			first = False
			time.sleep(7)
			parameter = "rosrun dynamic_reconfigure dynparam set /calibration_calculator bag " + str(0)
			os.system("gnome-terminal -e 'bash -c \"" + parameter + "\"'")
			os.system("gnome-terminal -e 'bash -c \"rostopic echo /calibration > distances.txt\"'")
		parameter1 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator N " + str(n)
		parameter2 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator r " + str(r)
		os.system("gnome-terminal -e 'bash -c \"" + parameter1 + "\"'")
		os.system("gnome-terminal -e 'bash -c \"" + parameter2 + "\"'")
		time.sleep(1)
		os.system("gnome-terminal -e 'bash -c \"cd bags/; rosbag play bag1.bag\"'")		
		print(str(counter) + "\n")
		counter = counter + 1				
		time.sleep(60)


parameter1 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator N " + str(n)
parameter2 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator r " + str(r)
os.system("gnome-terminal -e 'bash -c \"" + parameter1 + "\"'")
os.system("gnome-terminal -e 'bash -c \"" + parameter2 + "\"'")
time.sleep(1)

f = open('distances.txt', 'r')
f.readline()
f.readline()
f.readline()
f.readline()
f.readline()
f.readline()
for r in np.arange(0.073,0.08,0.001):
	for n in np.arange(41,45,1):
		result = f.readline()
		result1 = result[6:14]
		try:	
			res = float(result1)
			if(res < minValue):
				minValue = res
				params = [r, n]
		except ValueError:
			print("error"+ str(n) + " " + str(r))
		f.readline()
		f.readline()
		f.readline()
f.close
print(str(minValue) + " r: " + str(params[0]) + " n: " + str(params[1]))
