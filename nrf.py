import time
import RPi.GPIO as gpio
import serial
import json


ser=serial.Serial('/dev/ttyUSB0',9600)

gpio.setmode(gpio.BCM)

# pinouts
trig = 21
echo = 20
leftMotor = {"IN1": 14, "IN2":15}
rightMotor = {"IN1": 23, "IN2":18}

# pin setups
gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)

gpio.setup(leftMotor["IN1"], gpio.OUT)
gpio.setup(leftMotor["IN2"], gpio.OUT)
gpio.setup(rightMotor["IN1"], gpio.OUT)
gpio.setup(rightMotor["IN2"], gpio.OUT)


# The Ultrasonic sensor fonction
def ultrasonic():
    gpio.output(trig, 1)
    time.sleep(0.1)
    gpio.output(trig, 0)

    while gpio.input(echo) == False:
        start_time = time.time()

    while gpio.input(echo) == True:
        end_time = time.time()

    absolute_time_taken = end_time - start_time

    distance = absolute_time_taken / 0.0000582
    return distance


# The motor controller
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


# movements start
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
# movements End



if __name__ == "__main__":
    try:
        while True:
            result = ser.readline()

            r = result.split(b"{")
            r = r[1].decode()

            data = r.split('#')

            print(data[0])
            
            dis = ultrasonic()
            print("The distance is ", dis, "cm")
            if dis<35:
                # GPIO.output(led, 1)
                print("moving backward")
                moveBackward()
                time.sleep(1)
                forceStop()
                time.sleep(0.5)

                print("turning left")
                turnLeft()
                time.sleep(2)
                forceStop()
                time.sleep(0.5)
            else:
                # GPIO.output(led, 0)
                print("moving forward")
                moveForward()
        ser.close()
    except KeyboardInterrupt:
        gpio.cleanup()


