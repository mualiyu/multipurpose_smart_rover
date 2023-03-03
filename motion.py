import time
import RPi.GPIO as gpio

gpio.setmode(gpio.BCM)


leftMotor = {"IN1": 14, "IN2":15}
rightMotor = {"IN1": 23, "IN2":18}

gpio.setup(leftMotor["IN1"], gpio.OUT)
gpio.setup(leftMotor["IN2"], gpio.OUT)
gpio.setup(rightMotor["IN1"], gpio.OUT)
gpio.setup(rightMotor["IN2"], gpio.OUT)

def move(motor, direction):
    if direction == 0: #clockwise movement
        gpio.output(motor["IN1"], True)
        gpio.output(motor["IN2"], False)
    elif direction == 1:  #counterclockwise movement
        gpio.output(motor["IN1"], False)
        gpio.output(motor["IN2"], True)
    else:  #stop movement
        gpio.output(motor["IN1"], False)
        gpio.output(motor["IN2"], False)

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

gpio.cleanup()
