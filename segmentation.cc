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

void parsePoints(std::set<MapPoint*> points) {

    std::set<MapPoint*> axisPoints;

    std::set<MapPoint*>::iterator it = points.begin();
    for(it; it != points.end(); it++) {
        cv::Mat pos = it->getWorldPos();

        // get all the ones with the same z axis clustered
        // z_val = 

        // if there are multiple z axis' add it to the set 
        // axisPoints.insert(makePair(z_val, it))

    }
}