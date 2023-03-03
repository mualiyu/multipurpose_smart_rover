import time
import wiringpi

wiringpi.wiringPiSetupGpio()

leftMotor = {"IN1": 15, "IN2":16}
rightMotor = {"IN1": 4, "IN2":11}

wiringpi.pinMode(leftMotor["IN1"], 1)
wiringpi.pinMode(leftMotor["IN2"], 1)
wiringpi.pinMode(rightMotor["IN1"], 1)
wiringpi.pinMode(rightMotor["IN2"], 1)

def move(motor, direction):
    if direction == 0: #clockwise movement
        wiringpi.digitalWrite(motor["IN1"], 1)
        wiringpi.digitalWrite(motor["IN2"], 0)
    elif direction == 1:  #counterclockwise movement
        wiringpi.digitalWrite(motor["IN1"], 0)
        wiringpi.digitalWrite(motor["IN2"], 1)
    else:  #stop movement
        wiringpi.digitalWrite(motor["IN1"], 0)
        wiringpi.digitalWrite(motor["IN2"], 0)

def moveForward():
    move(rightMotor, 1)
    move(leftMotor, 1)

def moveBackward():
    move(rightMotor, 0)
    move(leftMotor, 0)

def forceStop():
    move(rightMotor, 2)
    move(leftMotor, 2)

def left():
    move(rightMotor, 1)
    move(leftMotor, 0)

def right():
    move(rightMotor, 0)
    move(leftMotor, 1)

def turnLeft():
    move(rightMotor, 1)
    move(leftMotor, 0)

def turnRight():
    move(rightMotor, 0)
    move(leftMotor, 1)

print("moving forward")
moveForward()
time.sleep(5)
forceStop()
time.sleep(1)
print("moving backward")
moveBackward()
time.sleep(5)
forceStop()
time.sleep(1)

print("turning left")
turnLeft()
time.sleep(3)
forceStop()
time.sleep(1)
print("turning right")
turnRight()
time.sleep(3)
forceStop()
time.sleep(1)

# print("moving left")
# left()
# time.sleep(3)
# forceStop()
# time.sleep(1)
# print("moving right")
# right()
# time.sleep(3)
# forceStop()
# forceStop()
# time.sleep(1)
# print("moving forward left")
# forwardLeft()
# time.sleep(3)
# forceStop()
# forceStop()
# time.sleep(1)
# print("moving forward right")
# forwardRight()
# time.sleep(3)
# forceStop()
# forceStop()
# time.sleep(1)
# print("moving backward left")
# backwardLeft()
# time.sleep(3)
# forceStop()
# forceStop()
# time.sleep(1)
# print("moving backward right")
# backwardRight()
# time.sleep(3)
# forceStop()
# forceStop()
# time.sleep(1)
# print("moving lateral Arc")
# lateralArc()
# time.sleep(3)
# forceStop()
# forceStop()
# time.sleep(1)
