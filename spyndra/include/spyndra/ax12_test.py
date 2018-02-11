import ax12, time
a = ax12.Ax12()
motor_id = 1

# id test
print 'ping id'
a.ping(motor_id)
a.scanServos(verbose=True)
print 'set id to 2'
a.setID(motor_id,2)
a.scanServos(verbose=True)
print 'reset motor'
a.factoryReset(2,confirm=True)
a.scanServos(verbose=True)

# read tests
print 'temperature: ' + str(a.readTemperature(motor_id))
print 'position: ' + str(a.readPosition(motor_id))
print 'voltage: ' + str(a.readVoltage(motor_id))
print 'present speed:' + str(a.readPresentSpeed(motor_id))
print 'goal speed:' + str(a.readGoalSpeed(motor_id))
print 'read load: ' + str(a.readLoad(motor_id))
print 'moving status:' + str(a.readMovingStatus(motor_id))
print 'reg-write instruction in use: ' + str(a.readRWStatus(motor_id))

# movement test
print 'move to 0'
a.move(motor_id,0)
while a.readMovingStatus(motor_id) == 1:
	pass
print 'move to 1023'
a.move(motor_id,1023)
while a.readMovingStatus(motor_id) == 1:
	pass
print 'move to 0 at speed 512'
a.moveSpeed(motor_id,0,512)
while a.readMovingStatus(motor_id) == 1:
	pass
print 'move to 1023 at speed 512'
a.moveSpeed(motor_id,1023,512)
while a.readMovingStatus(motor_id) == 1:
	pass

# reg-write movement test
a.moveRW(motor_id,0)
a.readRWStatus(motor_id)
a.action()
while a.readMovingStatus(motor_id) == 1:
	pass
a.moveSpeedRW(motor_id,1023,200)
a.readRWStatus(motor_id)
a.action()
while a.readMovingStatus(motor_id) == 1:
	pass
# accuracy test
print 'move to 400 at speed 300'
a.moveSpeed(motor_id,400,300)
print 'present speed:' + str(a.readPresentSpeed(motor_id))
while a.readMovingStatus(motor_id) == 1:
	pass
print 'goal speed:' + str(a.readGoalSpeed(motor_id))
print 'position: ' + str(a.readPosition(motor_id))