### Sending image data through lcm

mbot_img_streaming.py publish `mbot_image_t` lcm messages to send image array and `simple_motor_command_t` lcm messages to control the robot. These lcm types are defined in the **lcmtypes** folder. 

I compress the size of the image captured from the camera to 192*144. You can change that size if you want to test the streaming speed. 

The channel name I use to stream image data is `MBOT_IMAGE` and the one to stream control command is `MBOT_MOTOR_COMMAND_SIMPLE`.
