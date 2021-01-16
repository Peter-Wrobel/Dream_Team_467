#include "MobilePoint.h"
#include "MobileObject.h"
#include "ORBextractor.h"

#include "ORBmatcher.h"
#include "Frame.h"
#include "MapPoint.h"
#include <math.h>
#include <cmath>

#include<limits.h>
#include<utility>
#include<tuple>
#include<vector>


#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

    //std::vector<ORB_SLAM3::MapPoint*> 
    


using namespace ORB_SLAM3;



MobileObject::MobileObject(void){

    this->id = total_objects;
    total_objects++;

}




void MobileObject::UpdateMobileObjects(std::vector<MobileObject*>  & MobileObjects, std::vector<MobilePoint*> & MobilePoints){


    for (auto mo : MobileObjects){
        std::cout << " id is " << mo->id  << " number of points is " << mo->points.size()<< std::endl;

        std::tuple<double, double , double > v = mo->GetVel();
        std::cout << " velocity is ( " << std::get<0>(v) <<  ", " << std::get<1>(v) << ", " << std::get<2>(v) << " )" << std::endl;
    }


    // std::cout << "hey there crazy " << std::endl;
    // checks if new mobile point belongs to mobile object
    for(auto mo : MobileObjects){
        for (auto mb : MobilePoints){
            if(mb->dstate == MobilePoint::CANDIDATE) continue;

            if(mb->object_index == -1 &&  mo->PointIsInObject(mb)){

                // adding mobilepoint
                mb->object_index = mo->id;
                mo->points.push_back(mb);

            }

        }
    }
    // Makes new objects

    bool new_obj = true;
    // std::cout << "hey there crazy " << std::endl;

    while(new_obj){
        new_obj = false;

        for(auto mb_outer : MobilePoints){

            // Can't make objects out of candidates, or points in objects
            if(mb_outer->dstate ==MobilePoint::CANDIDATE || mb_outer->object_index != -1) continue;

            MobileObject *cand_ob = new MobileObject();
            cand_ob->points.push_back(mb_outer);


            for(auto mb_inner : MobilePoints){

                if(mb_inner->dstate ==MobilePoint::CANDIDATE || mb_inner->object_index != -1 || mb_inner == mb_outer) continue;

                if(mb_inner->object_index == -1 && cand_ob->PointIsInObject(mb_inner)){
                    cand_ob->points.push_back(mb_inner);
                }
            }
            auto vel = cand_ob->GetVel();
            if(cand_ob->points.size() >= 4 && sqrt(pow(std::get<0>(vel), 2)+  pow (std::get<1>(vel), 2))>0.01){
                std::cout << "making new object for us" << std::endl;
                MobileObjects.push_back(cand_ob);
                new_obj = true;

                for (auto mb_in_ob : cand_ob->points){
                    mb_in_ob->object_index = cand_ob->id;
                }
            }
            
            else{
                delete cand_ob;
                total_objects--;
            }
        }


    }

}

std::tuple<double, double, double> MobileObject::GetVel(){

    double points_size = (double)this->points.size();

    double x_v = 0;
    double y_v = 0;
    double z_v = 0;

    for(MobilePoint* point : this->points){

        auto one_v_both = point->CalcV();
        auto one_v = one_v_both.first;


        x_v+= std::get<0>(one_v);
        y_v+= std::get<1>(one_v);
        z_v+= std::get<2>(one_v);

    }

    return std::tuple<double, double, double>(x_v/points_size, y_v/points_size, z_v/points_size);

}

std::tuple<double, double, double> MobileObject::GetPos(){



    double points_size = (double)this->points.size();

    double x_v = 0;
    double y_v = 0;
    double z_v = 0;

    for(auto point : this->points){

        auto one_v = point->CurrentPos();


        x_v+= std::get<0>(one_v);
        y_v+= std::get<1>(one_v);
        z_v+= std::get<2>(one_v);

    }

    return std::tuple<double, double, double>(x_v/points_size, y_v/points_size, z_v/points_size);

}





bool MobileObject::PointIsInObject(MobilePoint * mb){

    auto mb_vel_pair = mb->CalcV();

    // Standard deviation is too high. We need to add more points
    if (mb_vel_pair.second> 3){
        return false;
    }

    // Tell if point is in object
    //cout << "=====ARE VELOCITIES SIM? " <<  this->SimilarVelocitiesObj(mb) << "ARE POSITIONS SIM ?" << this->SimilarPosObj(mb) << endl;
    return this->SimilarVelocitiesObj(mb) && this->SimilarPosObj(mb);

}


bool MobileObject::SimilarVelocitiesObj(MobilePoint * mb){

        auto one_v = mb->CalcV().first;
        auto two_v = this->GetVel();


        float x_pre = std::get<0>(one_v);
        float y_pre = std::get<1>(one_v);
        float z_pre = std::get<2>(one_v);

        float x_post = std::get<0>(two_v);
        float y_post = std::get<1>(two_v);
        float z_post = std::get<2>(two_v);

        return sqrt(pow(x_pre-x_post, 2) + pow(y_post-y_pre, 2) + pow(z_post + z_pre, 2)) < 0.11;
    
}


bool MobileObject::SimilarPosObj(MobilePoint * mb){


    double x_this, y_this, z_this;
    double x_that, y_that, z_that;

    std::tie(x_this, y_this, z_this) = this->GetPos();
    std::tie(x_that, y_that, z_that) = mb->CurrentPos();

    //cout << "SIMILAR POS CAL" <<  sqrt(pow(x_this-x_that, 2) + pow(y_this-y_that, 2) + pow(z_this - z_that,2)) << endl;

    return sqrt(pow(x_this-x_that, 2) + pow(y_this-y_that, 2) + pow(z_this - z_that,2)) < 0.16;




}











