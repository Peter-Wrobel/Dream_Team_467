#include "Atlas.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>
#include <lcm/lcm-cpp.hpp>
#include "./lcmtypes/mbot_ransac_t.hpp"

void parsePoints(std::set<ORB_SLAM3::MapPoint*> points, mbot_ransac_t pointList);

void parseMap(std::vector<ORB_SLAM3::KeyFrame*> frames);