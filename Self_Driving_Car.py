import cv2
import numpy as np
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
from matplotlib import pyplot as plt

width = 480
height = 480

camera = PiCamera()
camera.resolution = (width, height)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(width, height))

time.sleep(0.1)
GPIO.setmode(GPIO.BCM)

leftDC = 21
rightDC = 20
GPIO.setup(leftDC, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(rightDC, GPIO.OUT, initial=GPIO.LOW)

pwmleft = GPIO.PWM(leftDC, 100)
pwmright = GPIO.PWM(rightDC, 100)

Kp = 0.2
Kd = 1
# Ki = 0.01

previousTime = 0
currentError = 0
previousError = 0
derivative = 0
# integral = 0

currentTime = time.time()

currentX = width / 2
desiredX = currentX

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
    img_original = img
    # print("currentX is: ", currentX)

    img = cv2.GaussianBlur(img, (5, 5), 0)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret3, img1 = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((8, 8), np.uint8)
    img = cv2.morphologyEx(img1, cv2.MORPH_OPEN, kernel)

    y = int(0.85 * height)

    leftPoints = np.array([])
    rightPoints = np.array([])

    while y > 0:
        for x in range(width):
            if img[y, x] > 200:
                leftPoints = np.append(leftPoints, [y, x])
                break
        for x in range(width - 1, 0, -5):
            if img[y, x] > 200:
                rightPoints = np.append(rightPoints, [y, x])
                break
        y = int(y - 0.15 * height)

    leftPoints = leftPoints.astype('int32')
    rightPoints = rightPoints.astype('int32')

    if len(leftPoints) > 0:
        for i in range(0, len(leftPoints) - 2, 2):
            cv2.line(img_original, (leftPoints[i + 1], leftPoints[i]), (leftPoints[i + 3], leftPoints[i + 2]),
                     (255, 0, 0), 3)
            cv2.line(img_original, (rightPoints[i + 1], rightPoints[i]), (rightPoints[i + 3], rightPoints[i + 2]),
                     (0, 255, 255), 3)

    cv2.imshow('Camera Stream', img_original)

    if len(leftPoints) >= 4:
        desiredX = (leftPoints[3] + rightPoints[3]) / 2
    # print("desiredX is: ",desiredX)
    dT = (currentTime - time.time()) * 10
    print("dT is: ", dT)
    currentTime = time.time()
    # print("currentTime is: ",currentTime)
    # print("previousTime is: ",previousTime)
    currentError = currentX - desiredX
    # dT = currentTime - previousTime
    derivative = (currentError - previousError) / dT
    # integral = integral + currentError*dT

    speed = Kp * currentError + Kd * derivative
    print("current error is: ", currentError)
    print("derivative is: ", derivative)
    # print("integral is: ",integral)
    leftPwm = 50 + speed
    rightPwm = 65 - speed
    if leftPwm > 70:
        leftPwm = 70
    if rightPwm > 80:
        rightPwm = 80
    if leftPwm < 0:
        leftPwm = 0
    if rightPwm < 0:
        rightPwm = 0
    print("speed is: ", speed)
    print("leftPwm is: ", leftPwm)
    print("rightPwm is: ", rightPwm)

    pwmleft.start(leftPwm)
    pwmright.start(rightPwm)

    rawCapture.truncate(0)
    print("==============================")
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        pwmleft.start(0)
        pwmright.start(0)
        break


