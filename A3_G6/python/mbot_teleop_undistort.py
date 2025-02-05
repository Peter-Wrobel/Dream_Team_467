import pygame
from pygame.locals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
import os
from os.path import isfile, join
sys.path.append("lcmtypes")
import lcm
from lcmtypes import mbot_motor_pwm_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_video_stream_t
import json

#BeagleBone runs mobilebot (as shown in Piazza) and drive_simple (from A0)

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

pathIn= './video_images/'
pathOut = './undistort.mp4'

imu_path = './imu_data.csv'
target = open(imu_path, 'w+')
target.write("Timestamp(ns), gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z\n")

timestamp_path = './timestamps.txt'
time_file = open(timestamp_path, 'w+')

def lcm_handler():
    try:
        while True:
            lc.handle()
    except:
        pass
#lcm_thread = threading.Thread(target=lcm_handler, daemon=True)

def simple_imu_handler(channel, data):
    global target
    msg = mbot_imu_t.decode(data)
    if False: # and frame_count % 5 == 0:
        print(msg.accel)
    target.write(str(msg.utime) + ',' + str(msg.gyro[0]) + ',' + str(msg.gyro[1]) + ',' + str(msg.gyro[2]))
    target.write(',' + str(msg.accel[0]) + ',' + str(msg.accel[1]) + ',' + str(msg.accel[2]) + '\n')
imu_subscription = lc.subscribe("MBOT_IMU", simple_imu_handler)


config_file = 'cameraInfo'
for i in range(len(sys.argv)):
    if sys.argv[i] == '-f':
        config_file = sys.argv[i + 1]
config_file = 'cfg/' + config_file + '.json'

with open(config_file) as json_file:
    data = json.load(json_file)
    mtx  = np.array(data['intrinsics'])
    dist = np.array(data['distortion'])
    print("CAMERA INFO LOADED")
    print("Calibration Matrix:")
    print(mtx)
    print("Disortion:", dist)

vid_data = open('video_images/vid_data.csv', 'w+')
vid_data.write("#timestamp [ns],filename")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    lc.handle_timeout(10)
    image = frame.array
    screen.fill([0,0,0])
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    gray_image = gray_image.swapaxes(0,1)
    _, gray_encode = cv2.imencode(".png", gray_image)
    gray_encode = np.array(gray_encode)


    #image = cv2.flip(image, -1)
    
    # Undistort the image from the stream
    dim = image.shape[:2][::-1]
    
    timestamp_ns = time.time_ns()
    cv2.imwrite("video_images/" + str(timestamp_ns) +".png", image)
    time_file.write(str(timestamp_ns) + '\n')
    mapx,mapy = cv2.fisheye.initUndistortRectifyMap(mtx,dist,np.eye(3),mtx,dim, cv2.CV_16SC2)
    image = cv2.remap(image,mapx,mapy,interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
    # END undistortion attempt
    
    image = image.swapaxes(0,1)
    frame_num = "%08d" % (frame_count,)

    vid_data.write(str(time.time_ns()) + ',' + "frame_"+frame_num+".png")

    image = pygame.surfarray.make_surface(image)
    screen.blit(image, (0,0))
    pygame.display.update()

    frame_count += 1

    fwd = 0.0
    turn = 0.0
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            target.close()
            vid_data.close()
            time_file.close()
            pygame.quit()
            sys.exit()
            cv2.destroyAllWindows()
            
    key_input = pygame.key.get_pressed()  
    if key_input[pygame.K_LEFT]:
        turn += 1.0 #4.5 #uncommented values used with teleop_simple, others with teleop_simple_velocity
    if key_input[pygame.K_UP]:
        fwd += 1.0 #.25
    if key_input[pygame.K_RIGHT]:
        turn -= 1.0 #4.5
    if key_input[pygame.K_DOWN]:
        fwd -= 1.0 #.25
    if key_input[pygame.K_s]:
        #NOTE: Tutorial has the line below written twice in the image to vid function
        #I don't know what the purpose of that is, feel free to delete if it's unnecessary
        files = [f for f in os.listdir(pathIn) if isfile(join(pathIn, f))]
        files.sort(key = lambda x: x[5:-4])
        files.sort()
        for i in range(len(files)):
            filename=pathIn + files[i]
            #reading each files
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width,height)
            #inserting the frames into an image array
            frame_array.append(img)
        out = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'mp4v'), camera.framerate, size)
        for i in range(len(frame_array)):
            # writing to a image array
            out.write(frame_array[i])
        out.release()
    if key_input[pygame.K_q]:
        target.close()
        vid_data.close()
        time_file.close()
        pygame.quit()
        sys.exit()
        cv2.destroyAllWindows()
        

    command = mbot_motor_pwm_t()
    command.left_motor_pwm =  fwd * FWD_PWM_CMD - turn * TURN_PWM_CMD
    command.right_motor_pwm = fwd * FWD_PWM_CMD + turn * TURN_PWM_CMD
    image_stream = mbot_video_stream_t()
    image_stream.width = gray_image.shape[1]
    image_stream.height = gray_image.shape[0]
    image_stream.timestamp = timestamp_ns
    image_stream.enc_length = gray_encode.shape[0]
    image_stream.image = gray_encode
    lc.publish("MBOT_MOTOR_PWM",command.encode())
    lc.publish("MBOT_VIDEO_STREAM", image_stream.encode())
    rawCapture.truncate(0)
