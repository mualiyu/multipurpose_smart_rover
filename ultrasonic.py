import time
import RPi.GPIO as gpio
import serial

ser = serial.Serial("/dev/ttyUSB0", baudrate = 9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

gpio.setmode(gpio.BCM)

# pinouts
trig = 21
echo = 20
leftMotor = {"IN1": 14, "IN2":15}
rightMotor = {"IN1": 23, "IN2":18}
laser = 24

# pin setups
gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)

gpio.setup(leftMotor["IN1"], gpio.OUT)
gpio.setup(leftMotor["IN2"], gpio.OUT)
gpio.setup(rightMotor["IN1"], gpio.OUT)
gpio.setup(rightMotor["IN2"], gpio.OUT)

#laser pin setups
gpio.setup(laser, gpio.OUT)

# servor pin heads
gpio.setup(13, gpio.OUT)
# gpio.setup(12, gpio.OUT)
# servo1 = gpio.PWM(12, 50)
servo2 = gpio.PWM(13, 50)

# servo1.start(0)
servo2.start(0)
time.sleep(0.5)


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
            r = result.decode()
            r = r.split("{")
            r = r[1]
            print(r)
            # dffgh
            duty = 2
            #while r[2] == 1:
                #print("servo left")
                #servo2.ChangeDutyCycle(duty)
                #time.sleep(0.7)
                
                # servo1.ChangeDutyCycle(0)
                # time.sleep(0.7)
                #duty = duty + 1

            if r[0] == "1":  #operation is auto
                print("Automatic mode")
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

            elif r[0] == "0": # operation is manual
                print("manual Ops")
                if r[1] == "1": #move forward
                    print("Forward")
                    moveForward() 
                elif r[1] =="2": # backward
                    print("Backward")
                    moveBackward()
                elif r[1] =="3": # left
                    print("Right")
                    turnRight()
                elif r[1] =="4": # Right
                    print("Left")
                    turnLeft()
                else: #stop
                    print("rover stoped")
                    forceStop()
            
            if r[3] == "1":
                gpio.output(laser, 1)
            else:
                gpio.output(laser, 0)


            # servo controls
            
            if r[2] == "1":  #up
                servo2.ChangeDutyCycle(10)
                time.sleep(0.2)
            elif r[2] == "2": # down
                servo2.ChangeDutyCycle(12)
                time.sleep(0.2)
            # elif r[2] == "3": # down
            #     servo1.ChangeDutyCycle(2)
            #     time.sleep(0.2)
            # elif r[2] == "4": # down
            #     servo1.ChangeDutyCycle(12)
            #     time.sleep(0.2)
            # else:
            #     servo1.ChangeDutyCycle(7.5)
            #     time.sleep(0.2)
                
    except KeyboardInterrupt:
        gpio.cleanup()


