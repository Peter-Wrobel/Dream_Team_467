import pygame
from pygame.locals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
import os
sys.path.append("../lcmtypes")
import lcm
from lcmtypes import mbot_motor_pwm_t
from lcmtypes import simple_motor_command_t
from lcmtypes import mbot_imu_t
#import threading

FWD_PWM_CMD = 0.3
TURN_PWM_CMD = 0.3

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

pygame.init()
pygame.display.set_caption("MBot TeleOp")
screen = pygame.display.set_mode([640,480])
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.5)
frame_count = 0

def lcm_handler():
    try:
        while True:
            lc.handle()
    except:
        pass
#lcm_thread = threading.Thread(target=lcm_handler, daemon=True)

def simple_imu_handler(channel, data):
    msg = mbot_imu_t.decode(data)
    if False: # and frame_count % 5 == 0:
        print(msg.accel)
imu_subscription = lc.subscribe("MBOT_IMU", simple_imu_handler)

#lcm_thread.start()
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    lc.handle()
    image = frame.array
    screen.fill([0,0,0])
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    if False and frame_count % 5 == 0:
        cv2.imwrite("images/calibration_image_"+str(frame_count//10)+".jpg", image)


    image = image.swapaxes(0,1)
    image = pygame.surfarray.make_surface(image)
    screen.blit(image, (0,0))
    pygame.display.update()

    frame_count += 1

    fwd = 0.0
    turn = 0.0
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            pygame.quit()
            sys.exit()
            cv2.destroyAllWindows()
    key_input = pygame.key.get_pressed()  
    if key_input[pygame.K_LEFT]:
        turn += 4.5
    if key_input[pygame.K_UP]:
        fwd += 0.25
    if key_input[pygame.K_RIGHT]:
        turn -= 4.5
    if key_input[pygame.K_DOWN]:
        fwd -= 0.25
    if key_input[pygame.K_q]:
        pygame.quit()
        sys.exit()
        cv2.destroyAllWindows()
    #command = simple_motor_command_t.simple_motor_command_t() # mbot_motor_pwm_t()
    command = simple_motor_command_t() # mbot_motor_pwm_t()
    #command.left_motor_pwm =  fwd * FWD_PWM_CMD - turn * TURN_PWM_CMD
    #command.right_motor_pwm = fwd * FWD_PWM_CMD + turn * TURN_PWM_CMD
    command.forward_velocity = fwd
    command.angular_velocity = turn
    lc.publish("MBOT_MOTOR_COMMAND_SIMPLE",command.encode())

    rawCapture.truncate(0)
