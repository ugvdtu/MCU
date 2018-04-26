import rospy
from std_msgs.msg import Int8MultiArray, UInt8MultiArray
from time import sleep

FORWARD = 1
BACKWARD = 0
SLOW = 100
FAST = 200

PUBLISHER_TOPIC = 'MCU_Input'
SUBSCRIBER_TOPIC = 'IMU_Output'

class Navigator:
	
	def __init__(self):
		rospy.init_node('Navigator', anonymous=True)
		self.publisher = rospy.Publisher(PUBLISHER_TOPIC, UInt8MultiArray, queue_size=5)
		self.subscriber = rospy.Subscriber(SUBSCRIBER_TOPIC, Int8MultiArray, self.update_angle)
		self.angle = 0
		self.r = rospy.Rate(10)
		sleep(3)          # Time to properly initialize stuff

	def update_angle(self, new_angle):
		self.angle = int(new_angle.data[3])

	def motor_cmd(self, left_dir, left_speed, right_dir, right_speed):
		cmd = UInt8MultiArray(data=[left_dir, left_speed, right_dir, right_speed])
		self.publisher.publish(cmd)
		print 'Published %r' % cmd.data
		self.r.sleep()

	def turn_left(self, target_angle):
		while self.angle < target_angle:
			self.motor_cmd(FORWARD, 0, FORWARD, SLOW)

	def turn_right(self, target_angle):
		while self.angle > target_angle:
			self.motor_cmd(FORWARD, SLOW, FORWARD, 0)

	def move_forward(self, speed):
		self.motor_cmd(FORWARD, speed, FORWARD, speed)

	def move_backward(self, speed):
		self.motor_cmd(BACKWARD, speed, BACKWARD, speed)

	def turn(self, target_angle):
		"""Negative angle = right turn
		   and positive angle = left turn"""
		if self.angle < target_angle:
			self.turn_left(target_angle)
		else:
			self.turn_right(target_angle)