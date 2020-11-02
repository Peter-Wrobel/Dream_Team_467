LCM = lcm-gen
LCMTYPES_NAMES := $(shell cd ../lcmtypes && ls *.lcm)
LCMTYPES_HPP := $(LCMTYPES_NAMES:%.lcm=%.hpp)
ALL = $(LCMTYPES_HPP)

all: $(ALL)
	g++ -o vid_stream vid_stream.cpp -I/usr/local/include -I/usr/local/include/opencv -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -L/usr/local/lib -llcm -lglib-2.0 -lopencv_dnn -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_features2d -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core



%.hpp:../lcmtypes/%.lcm
	$(LCM) -x --cpp-hpath ./lcmtypes $<

clean:
	rm -rf lcmtypes/
	rm vid_stream
