#ifndef MOBILE_OBJ_H
#define MOBILE_OBJ_H

#include "Frame.h"
#include "MapPoint.h"
#include "MobilePoint.h"
#include <tuple>
#include <utility>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#define OBJ_THRESH 5

class MobileObject{

public:

MobileObject (void);



// Gets rid of weakest object/adds objects
static void UpdateMobileObjects(std::vector<MobileObject*>  & MobileObjects, std::vector<MobilePoint*> & MobilePoints);

static int total_objects;


bool PointIsInObject(MobilePoint * mb);


std::tuple<double, double, double> GetVel();

std::tuple<double, double, double> GetPos();



bool SimilarVelocitiesObj(MobilePoint * mb);

bool SimilarPosObj(MobilePoint * mb);



// identification.
int id;


vector<MobilePoint *> points;


};









#endif //MOBILE_OBJ_H