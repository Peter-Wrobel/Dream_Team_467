#include "segmentation.h"


void parsePoints(std::set<ORB_SLAM3::MapPoint*> points, mbot_ransac_t pointList) {

    std::set<ORB_SLAM3::MapPoint*>::iterator it = points.begin();
    float worldPoints[points.size()*3];

    size_t index = 0;
    for(it; it != points.end(); ++it) {
        cv::Mat pos = (*it)->GetWorldPos();
        worldPoints[index] = pos.at<float>(0);
        worldPoints[index+1] = pos.at<float>(1);
        worldPoints[index+2] = pos.at<float>(2);
        index = index + 3;
    }

    std::copy(worldPoints, worldPoints+(points.size()*3), std::back_inserter(pointList.points));
    pointList.num_points += points.size() * 3;
}

// using an input of KeyFrames, we will parse through all of the points
// and determine what would be the different planes from the static
// points. 
void parseMap(std::vector<ORB_SLAM3::KeyFrame*> frames) {
    // add in initial conditions before determining the plane bc RANSAC
    // is computationally expensive

    // from each KeyFrame, we want to get each of the map points
    std::cout << "In the map\n";
    mbot_ransac_t pointList;
    pointList.num_points = 0;

    for(size_t i = 0; i < frames.size(); ++i) {
        std::set<ORB_SLAM3::MapPoint*> points = frames[i]->GetMapPoints();
        parsePoints(points, pointList);
    } 

    lcm::LCM lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    if(!lcm.good()){
        std::cout << "Failed in segmentation.cc\n";
        return;
    }
    lcm.publish("MBOT_RANSAC_SEND", &pointList);
}