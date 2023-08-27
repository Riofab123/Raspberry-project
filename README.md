# Raspberry-project
#Integrate Raspberry Pi cameras to detect lines or take turns based  on colours
import numpy as np
import RPi.GPIO as GPIO
import cv2
import time
cap = cv2.VideoCapture(0)
cap.set(3, 160)
cap.set(4, 120)
cap.set(cv2.CAP_PROP_FPS, 20)
fps = int(cap.get(5))
print("fps:",fps)
in1 = 3 #17
in2 = 5 #27
in3 = 29 #5
in4 = 31 #6
enA = 32
enB = 33
c=""
GPIO.setmode(GPIO.BOARD)
GPIO.setup(enA, GPIO.OUT)
GPIO.setup(enB, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
p1 = GPIO.PWM(enA, 30)
p2 = GPIO.PWM(enB, 30)
p1.start(20)
p2.start(20)
def forward():
GPIO.output(in1, GPIO.HIGH)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.HIGH)
GPIO.output(in4, GPIO.LOW)
p1.start(18)
p2.start(18)
def backward():
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.HIGH)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.HIGH)
def left():
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.HIGH)
GPIO.output(in3, GPIO.HIGH)
GPIO.output(in4, GPIO.LOW)
p1.start(12)
p2.start(15 )
def right():
GPIO.output(in1, GPIO.HIGH)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)
p1.start(15)
p2.start(12)
def stop():
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)
stop()
try:
while True:
ret, frame = cap.read()
hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# Set range for red color and
# define mask
red_lower = np.array([57,69,68], np.uint8)
red_upper = np.array([108, 255,255], np.uint8)
red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
# Set range for green color and
# define mask
green_lower = np.array([46, 127, 79], np.uint8)
green_upper = np.array([92, 255, 255], np.uint8)
green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
# Set range for blue color and
# define mask
blue_lower = np.array([79, 99, 121], np.uint8)
blue_upper = np.array([123, 255, 255], np.uint8)
blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
kernel = np.ones((5, 5), "uint8")
# For red color
red_mask = cv2.dilate(red_mask, kernel)
res_red = cv2.bitwise_and(frame, frame,
mask = red_mask)
# For green color
green_mask = cv2.dilate(green_mask, kernel)
res_green = cv2.bitwise_and(frame, frame,
mask = green_mask)
# For blue color
blue_mask = cv2.dilate(blue_mask, kernel)
res_blue = cv2.bitwise_and(frame, frame,
mask = blue_mask)
low_b = np.uint8([80,80,80])
high_b = np.uint8([0,0,0])
mask = cv2.inRange(frame, high_b, low_b)
contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
if len(contours) > 0 :
c = max(contours, key=cv2.contourArea)
M = cv2.moments(c)
if M["m00"] !=0 :
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
print("CX : "+str(cx)+" CY : "+str(cy))
if cx >= 120 :
print("Turn Right")
right()
if cx < 120 and cx > 40 :
print("On Track!")
forward()
if cx <=40 :
print("Turn Left")
left()
cv2.circle(frame, (cx,cy), 5, (255,255,255), -1)
else :
print("I don't see the line")
stop()
time.sleep(1)
# Creating contour to track green color
contours, hierarchy = cv2.findContours(green_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
for pic, contour in enumerate(contours):
area = cv2.contourArea(contour)
if(area > 300):
print("Green Detected")
p1.ChangeDutyCycle(10)
p2.ChangeDutyCycle(10)
left()
time.sleep(0.5)
# Creating contour to track blue color
contours,hierarchycv2.findContours(blue_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
for pic, contour in enumerate(contours):
area = cv2.contourArea(contour)
if(area > 300):
print("Blue Detected")
forward()
time.sleep(14)
stop()
#time.sleep(5)
GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()
# Creating contour to track red color
contours, hierarchy = cv2.findContours(red_mask,
cv2.RETR_TREE,
cv2.CHAIN_APPROX_SIMPLE)
for pic, contour in enumerate(contours):
area = cv2.contourArea(contour)
if(area > 300):
print("Red Detected")
p1.ChangeDutyCycle(10)
p2.ChangeDutyCycle(10)
right()
time.sleep(0.7)
cv2.drawContours(frame, c, -1, (0,255,0), 1)
cv2.imshow("Mask",mask)
cv2.imshow("Frame",frame)
if cv2.waitKey(1) & 0xff == ord('q'): # 1 is the time in ms
stop()
break
finally:
GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()

