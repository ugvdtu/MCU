import matplotlib.pyplot as plt
import numpy
import rospy
import os
from std_msgs.msg import Int8MultiArray

def on_imu(raw_data):
	plt.subplot(111, projection='polar')	
	plt.cla()


	plt.plot([0, numpy.deg2rad(raw_data.data[3])], [0, 5], 'b')
	plt.show()
	plt.pause(0.00000000001)

def main():
	plt.ion()

	rospy.init_node('IMU_plotter', anonymous=True)
	r = rospy.Subscriber("MCU_Output", Int16, on_imu)
	print "PRESS CTRL-C TO QUIT"
	rospy.spin()
	os._exit(0)

if __name__ == '__main__':
	main()