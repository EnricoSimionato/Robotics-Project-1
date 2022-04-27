import os

########################################
#       Da mettere in ~/robotics       #
########################################


minValue = -1
for r in range(1,2,1):
	for n in range(1,2,1):
		for w in range(1,2,1):
			for l in range(1,2,1):
				os.system("gnome-terminal -e 'bash -c \"rosparam set /odometry_calculator/method RK; catkin_make; cd src/Robotics-Project-1; roslaunch project1 project1.launch; exec bash\"'")
				os.system("gnome-terminal -e 'bash -c \"cd src/Robotics-Project-1/bags/; rosbag play bag1.bag; exec bash\"'")
				# Potremmo scrivere su un file e poi leggere e parsare
				# Usare a metrica appropriata e fare il calcolo
				# Trovare un modo per killare i processi
				actualValue = 0
				if(minValue == -1):
					minValue = 0; # Valore restituito
				else:
					if(actualValue < minValue):
						parameters = [r, n, w, l]