"""import os
import numpy as np
import time
import sys

#per il movimento lineare basta r e n

minValue = 1000000
first = True
os.system("gnome-terminal -e 'bash -c \" catkin_make; roslaunch project1 project1.launch; exec bash\"'")
with open('distances.txt', 'w') as f:
	sys.stdout = f;
	for r in np.arange(0.07,0.08,0.01):
		for n in np.arange(42, 45,1):
			print(str(n) + " " + str(r))
			if first:
				first = False
				time.sleep(10)
				os.system("gnome-terminal -e 'bash -c \"rostopic echo /calibration > distances.txt\"'")
			parameter1 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator N " + str(n)
			parameter2 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator r " + str(r)
			os.system("gnome-terminal -e 'bash -c \"" + parameter1 + "\"'")
			os.system("gnome-terminal -e 'bash -c \"" + parameter2 + "\"'")
			time.sleep(1)
			os.system("gnome-terminal -e 'bash -c \"cd bags/; rosbag play -u 10 bag1.bag; exec bash\"'")							
			time.sleep(10)
	f.close


# Trovare un modo per killare i processi
actualValue = 0
if(minValue == -1):
	minValue = 0; # Valore restituito
else:
	if(actualValue < minValue):
		parameters = [r, n, w, l]"""

import os
import numpy as np
import time
import sys

#per il movimento lineare basta r e n

minValue = 1000000.0
counter = 0
first = True
os.system("gnome-terminal -e 'bash -c \" catkin_make; roslaunch project1 project1.launch; exec bash\"'")
for r in np.arange(0.065,0.075,0.001):
	for n in np.arange(38,46,1):
		if first:
			first = False
			time.sleep(7)
			os.system("gnome-terminal -e 'bash -c \"rostopic echo /calibration > distances.txt\"'")
		parameter1 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator N " + str(n)
		parameter2 = "rosrun dynamic_reconfigure dynparam set /calibration_calculator r " + str(r)
		os.system("gnome-terminal -e 'bash -c \"" + parameter1 + "\"'")
		os.system("gnome-terminal -e 'bash -c \"" + parameter2 + "\"'")
		time.sleep(1)
		os.system("gnome-terminal -e 'bash -c \"cd bags/; rosbag play -s 1 -u 20 bag1.bag; exec bash\"'")		
		print(str(counter) + "\n")
		counter = counter + 1				
		time.sleep(21)

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
for r in np.arange(0.065,0.075,0.001):
	for n in np.arange(38,46,1):
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
# Trovare un modo per killare i processi

