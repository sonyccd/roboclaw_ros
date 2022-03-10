import time

import roboclaw_driver as roboclaw

#Windows comport name
#rc_front = Roboclaw("COM3",115200)
#Linux comport name

rc_back = roboclaw("/dev/ttyACM3",115200)
rc_front = roboclaw("/dev/ttyACM4", 115200)

def displayspeed():
	enc1_front = rc_front.ReadEncM1(address_front)
	enc2_front = rc_front.ReadEncM2(address_front)
	speed1_front = rc_front.ReadSpeedM1(address_front)
	speed2_front = rc_front.ReadSpeedM2(address_front)
	enc1_back = rc_back.ReadEncM1(address_back)
	enc2_back = rc_back.ReadEncM2(address_back)
	speed1_back = rc_back.ReadSpeedM1(address_back)
	speed2_back = rc_back.ReadSpeedM2(address_back)

	print("Encoder1_front:"),
	if(enc1_front[0]==1):
		print enc1_front[1],
		print format(enc1_front[2],'02x'),
	else:
		print "failed",
	print "Encoder2_front:",
	if(enc2_front[0]==1):
		print enc2_front[1],
		print format(enc2_front[2],'02x'),
	else:
		print "failed " ,
	print "Speed1_front:",
	if(speed1_front[0]):
		print speed1_front[1],
	else:
		print "failed",
	print("Speed2_front:"),
	if(speed2_front[0]):
		print speed2_front[1]
	else:
		print "failed "

	print("Encoder1_back:"),
	if(enc1_back[0]==1):
		print enc1_back[1],
		print format(enc1_back[2],'02x'),
	else:
		print "failed",
	print "Encoder2_back:",
	if(enc2_back[0]==1):
		print enc2_back[1],
		print format(enc2_back[2],'02x'),
	else:
		print "failed " ,
	print "Speed1_back:",
	if(speed1_back[0]):
		print speed1_back[1],
	else:
		print "failed",
	print("Speed2_back:"),
	if(speed2_back[0]):
		print speed2_back[1]
	else:
		print "failed "

rc_front.Open()
rc_back.Open()
address_front = 0x80
address_back = 0x81

version_front = rc_front.ReadVersion(address_front)
if version_front[0]==False:
	print "GETVERSION_front Failed"
else:
	print repr(version_front[1])

version_back = rc_back.ReadVersion(address_back)
if version_back[0]==False:
	print "GETVERSION_back Failed"
else:
	print repr(version_back[1])

while(1):
	rc_front.SpeedM1(address_front,12000)
	rc_front.SpeedM2(address_front,-12000)
	rc_back.SpeedM1(address_back,12000)
	rc_back.SpeedM2(address_back,-12000)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)

	rc_front.SpeedM1(address_front,-12000)
	rc_front.SpeedM2(address_front,12000)
	rc_back.SpeedM1(address_back,-12000)
	rc_back.SpeedM2(address_back,12000)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)