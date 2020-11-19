
#ifndef MOBILE_POINT_H
#define MOBILE_POINT_H

#include "Frame.h"
#include "MapPoint.h"

class MobilePoints{

    public:
    static vector<ORB_SLAM3::MapPoint*> SearchForMobile(ORB_SLAM3::Frame &CurrentFrame, const ORB_SLAM3::Frame &LastFrame, const float th, const bool bMono);





};


#endif 