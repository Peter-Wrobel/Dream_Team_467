#include "segmentation.h"

// using an input of KeyFrames, we will parse through all of the points
// and determine what would be the different planes from the static
// points. 
void parseMap(std::vector<KeyFrame*> frames) {
    // add in initial conditions before determining the plane bc RANSAC
    // is computationally expensive

    // from each KeyFrame, we want to get each of the map points
    std::vector<KeyFrame*>::iterator it = frames.begin();

    for(it; it != frames.end(); it++) {
        std::set<MapPoint*> points = frames.GetMapPoints();
        std::set<MapPoint*> axisPoints = parsePoints(points);

        // for each cluster of points from axisPoints, determine if it's the ground plane

    } 
    
}

float* parsePoints(std::set<MapPoint*> points) {

    float[points.size()][3] worldPoints;

    std::set<MapPoint*>::iterator it = points.begin();
    size_t index = 0;
    for(it; it != points.end(); it++) {
        cv::Mat pos = it->getWorldPos();

        // get all the ones with the same z axis clustered
        worldPoints[index][0] = pos.at<float>(0);
        worldPoints[index][1] = pos.at<float>(1);
        worldPoints[index][2] = pos.at<float>(2);

        // if there are multiple z axis' add it to the set 
        index++;
    }

    return worldPoints;
}
