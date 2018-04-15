from time import sleep
import navigate as nav
 
nav.init()
INTERVAL = 5 #seconds

# Helper functions

def forward_backward(speed_f, speed_b):
	nav.move_forward(speed_f)
	sleep(INTERVAL)
	nav.move_backward(speed_b)
	sleep(INTERVAL)

def turn_while_moving_forward(speed, angle):
	nav.move_forward(speed)
	sleep(INTERVAL)
	nav.turn(angle)



# The tests

def forward_backward_slow():
	forward_backward(nav.SLOW, nav.SLOW)

def forward_backward_fast():
	forward_backward(nav.FAST, nav.FAST)

def forward_backward_slow_fast():
	nav.forward_backward(nav.SLOW, nav.FAST)

def backward_forward_fast_slow():
	nav.forward_backward(nav.FAST, nav.SLOW)

def turn_while_moving_fast():
	nav.turn_while_moving_forward(nav.FAST, 0)

def turn_while_moving_slow():
	nav.turn_while_moving_forward(nav.SLOW, 0)


