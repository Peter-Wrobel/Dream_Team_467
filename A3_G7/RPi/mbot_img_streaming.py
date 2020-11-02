import pygame
from pygame.locals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
import time
sys.path.append("lcmtypes")
import lcm
#from lcmtypes import simple_motor_command_t
from simple_motor_command_t import *
from mbot_image_t import *


FWD_PWM_CMD = 0.25
TURN_PWM_CMD = 0.25
flip_h = 0
flip_v = 0

# ---------------------------------------------------------
# TODO: feel free to change the size of the image you want 
# and remember to change the image size in mbot_img_t.lcm
# ---------------------------------------------------------
width = 192
height = 144
# ---------------------------------------------------------

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("MBot Tracking")
screen = pygame.display.set_mode([width,height])
camera = PiCamera()
camera.resolution = (width, height)
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=(width, height))
time.sleep(0.5)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    if (flip_h == 1 & flip_v == 0):
        image = cv2.flip(image, 1)
    elif (flip_h == 0 & flip_v == 1):
        image = cv2.flip(image, 0)
    elif (flip_h == 1 & flip_v == 1):
        image = cv2.flip(image, -1)
    
    timestamp = int(time.time() * 1000)
    img_msg = mbot_image_t()
    img_msg.utime = timestamp
    img_msg.encode_img = image.tolist()
    # print(image.shape)
    lc.publish("MBOT_IMAGE", img_msg.encode())

    screen.fill([0,0,0])
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.swapaxes(0,1)
    image = cv2.flip(image, -1)
    image = pygame.surfarray.make_surface(image)
    screen.blit(image, (0,0))
    pygame.display.update()
    fwd = 0.0
    turn = 0.0
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            pygame.quit()
            sys.exit()
            cv2.destroyAllWindows()
    key_input = pygame.key.get_pressed()  
    if key_input[pygame.K_LEFT]:
        turn += 1.0
    if key_input[pygame.K_UP]:
        fwd +=1.0
    if key_input[pygame.K_RIGHT]:
        turn -= 1.0
    if key_input[pygame.K_DOWN]:
        fwd -= 1.0
    if key_input[pygame.K_h]:
        if flip_h == 0:
            flip_h = 1
        else:
            flip_h = 0
    if key_input[pygame.K_v]:
        if flip_v == 0:
            flip_v = 1
        else:
            flip_v = 0
    if key_input[pygame.K_c]:
        camera.capture('signal.jpg')
    if key_input[pygame.K_q]:
            pygame.quit()
            time_file.close()
            sys.exit()
            cv2.destroyAllWindows()
    
    left_motor_pwm =  fwd * FWD_PWM_CMD - turn * TURN_PWM_CMD
    right_motor_pwm = fwd * FWD_PWM_CMD + turn * TURN_PWM_CMD
    
    command = simple_motor_command_t()
    command.forward_velocity = (left_motor_pwm + right_motor_pwm) / 2.0
    command.angular_velocity = (right_motor_pwm - left_motor_pwm) / 12.0
    
    lc.publish("MBOT_MOTOR_COMMAND_SIMPLE",command.encode())
    rawCapture.truncate(0)
