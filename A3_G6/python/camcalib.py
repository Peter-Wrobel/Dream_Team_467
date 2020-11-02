#!/usr/bin/env python

"""
From https://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
"""

import numpy as np
import matplotlib.pyplot as plt
import cv2
import glob
import sys
import os
import json


#---------------------- SET THE PARAMETERS
nRows = 9
nCols = 6
dimension = 25 #- mm

for i in range(len(sys.argv)):
    if sys.argv[i] == '-r':
        nRows = int(sys.argv[i + 1])
    if sys.argv[i] == '-c':
        nCols = int(sys.argv[i+1])
    if sys.argv[i] == '-d':
        dimension = float(sys.argv[i + 1])

workingFolder   = "./images"
imageType       = 'jpg'
#------------------------------------------

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, dimension, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((nRows*nCols,3), np.float32)
objp[:,:2] = np.mgrid[0:nCols,0:nRows].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Find the images files
filename    = workingFolder + "/*." + imageType
images      = glob.glob(filename)

print("Found", len(images), "images")
if len(images) < 15:
    print("Not enough images were provided. We need at least 15 images for a" \
          " good calibration attempt")
    sys.exit()

else:
    nPatternFound = 0
    img_to_undistort = images[0]

    for fname in images:
        if 'calibresult' in fname: continue
        #-- Read the file and convert in greyscale
        img     = plt.imread(os.path.join(os.getcwd(), fname[2:]))
        gray    = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)

        print("Reading image ", fname)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (nCols,nRows),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            print("Pattern found! Press ESC to skip or ENTER to accept")
            #--- Sometimes, Harris corners fails with crappy pictures, so
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (nCols,nRows), corners2,ret)
            cv2.imshow('img',img)
            # cv2.waitKey(0)
            k = cv2.waitKey(0) & 0xFF
            if k == 27: #-- ESC Button
                print("Image Skipped")
                img_to_undistort = fname
                continue

            print("Image accepted")
            nPatternFound += 1
            objpoints.append(objp)
            imgpoints.append(corners2)

            # cv2.waitKey(0)
        else:
            img_to_undistort = fname


cv2.destroyAllWindows()

if (nPatternFound > 9):
    print("Found %d good images" % (nPatternFound))
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    """
    # Undistort an image
    img = plt.imread(img_to_undistort)
    img = img[..., ::-1]  # RGB --> BGR
    h,  w = img.shape[:2]
    print("Image to undistort: ", img_to_undistort)
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    # undistort
    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    print("ROI: ", x, y, w, h)

    cv2.imwrite(workingFolder + "/calibresult.jpg",dst)
    print("Calibrated picture saved as calibresult.jpg")"""
    print("Calibration Matrix:")
    print(mtx)
    print("Disortion:", dist)

    #--------- Save result in json file
    # make camera matrix and distortion vector into dictionary
    mtx2 = mtx.tolist()
    dist2 = dist.tolist()
    data = {
        "intrinsics": mtx2,
        "distortion": dist2
    }
    with open('cameraInfo.json', 'w') as json_file:
        json.dump(data, json_file)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    print("total error: ", mean_error/len(objpoints))

else:
    print("No calibration occurred because less than 10 images had correct patterns")
    print("In order to calibrate you need at least 10 correct pattern identifications")
    print("Take some more pictures and try again")

