    
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
#include <vector>
#include <tuple>


#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

    //std::vector<ORB_SLAM3::MapPoint*> 
    

// using namespace ORB_SLAM3::

using namespace ORB_SLAM3;


MobilePoint::MobilePoint(const cv::KeyPoint kp, const cv::Mat desc, const cv::Mat pos, double t_delt ){
    name_keyp = kp;
    name_desc = desc;
    place_history_3d.push_back(pos.clone());
    time_history.push_back(t_delt);
    looked_for = 0;
    seen = 0;
    object_index = -1;
}

void MobilePoint::PrintHiredInfo(std::vector<MobilePoint*> & MobilePoints){
    // for (auto mb : MobilePoints){
        
    //     if(mb->dstate == CANDIDATE || mb->object_index == -1) continue;

    //     cout << "=====MobilePoint=====\n";

    //     cout << "POINTS: \n" ;
    //     vector<float > velocos_x; 
    //     vector<float > velocos_y; 
    //     vector<float > velocos_z; 


    //     for(int inner = 1; inner < (int) mb->place_history_3d.size(); ++inner){


    //         cv::Mat threed_pre = mb->place_history_3d[inner-1];
    //         cv::Mat threed_post = mb->place_history_3d[inner]; 

    //         float x_pre = threed_pre.at<float>(0);
    //         float y_pre = threed_pre.at<float>(1);
    //         float z_pre = threed_pre.at<float>(2);

    //         float x_post = threed_post.at<float>(0);
    //         float y_post = threed_post.at<float>(1);
    //         float z_post = threed_post.at<float>(2);

    //          cout << "(" << x_pre << "," << y_pre << "," << z_pre << ")\n";

    //         velocos_x.push_back(x_post-x_pre);
    //         velocos_y.push_back(y_post - y_pre);
    //         velocos_z.push_back(z_post-z_pre);



    //     }

    //     cout << "\n VELOCITIES:\n";


    //     for (int inner = 0; inner < (int) velocos_z.size(); ++inner ){

    //         float x_pre = velocos_x[inner];
    //         float y_pre = velocos_y[inner];
    //         float z_pre = velocos_z[inner];
                         
    //         cout << "(" << x_pre << "," << y_pre << "," << z_pre << ")\n";

    //     }


    //     cout << "=============\n";
        
    // }
}


std::pair<std::tuple<double,double, double>, double> MobilePoint::CalcV( int starty){

    vector<double > velocos_x; 
    vector<double > velocos_y; 
    vector<double > velocos_z; 

    double av_x = 0;
    double av_y = 0;
    double av_z = 0;

    int total_occ = (int) place_history_3d.size() - starty-1;

    //std::cout << "inc alc vccc" << std::endl;
    if (total_occ == 0){
        total_occ = 1;
    }


    for(int inner = starty+ 1; inner < (int) place_history_3d.size(); ++inner){


        cv::Mat threed_pre = place_history_3d[inner-1];
        cv::Mat threed_post = place_history_3d[inner]; 

        float x_pre = threed_pre.at<float>(0);
        float y_pre = threed_pre.at<float>(1);
        float z_pre = threed_pre.at<float>(2);

        float x_post = threed_post.at<float>(0);
        float y_post = threed_post.at<float>(1);
        float z_post = threed_post.at<float>(2);

        double time_d = time_history[inner] - time_history[inner-1];
        //std::cout << "time di is " << time_d << std::endl;
    


        velocos_x.push_back(double(x_post-x_pre)/time_d);
        velocos_y.push_back(double(y_post - y_pre)/time_d);
        velocos_z.push_back(double(z_post-z_pre)/time_d);

        av_x += double(x_post-x_pre)/time_d;
        av_y += double(y_post - y_pre)/time_d;
        av_z += double(z_post-z_pre)/time_d;

    }
   // std::cout << "inc alc v" << std::endl;

    av_x /= (double)total_occ;
    av_y /= (double)total_occ;
    av_z /= (double)total_occ;


    double stand_dev_x = 0;
    double stand_dev_y = 0;
    double stand_dev_z = 0;

    for (int inner = 0; inner < (int) velocos_x.size(); ++inner){

        stand_dev_x += pow(velocos_x[inner] - av_x , 2);
        stand_dev_y += pow(velocos_y[inner] - av_y , 2);
        stand_dev_z += pow(velocos_z[inner] - av_z , 2);
    }

    stand_dev_x = sqrt(stand_dev_x/(double)(total_occ));
    stand_dev_y = sqrt(stand_dev_y/(double)(total_occ));
    stand_dev_z = sqrt(stand_dev_z/(double)(total_occ));


    double stand_dev = (stand_dev_x) + (stand_dev_y) + (stand_dev_z);

    std::tuple<double,double,double> av_v (av_x, av_y, av_z);

    if(av_x >10|| av_x < -10 || av_y <-10 || av_z<-10|| av_y >10|| av_z>10 ){
        std::cout << "ya its this issue" << av_x << " " << av_y  << " av_z " << std::endl;
        //exit(1);
    }

    return std::pair< std::tuple<double,double,double>, double > (av_v, stand_dev);


}





bool MobilePoint::SimilarVelocities(MobilePoint* mb){

        auto one_v = mb->CalcV().first;
        auto two_v = this->CalcV().first;


        double x_pre = std::get<0>(one_v);
        double y_pre = std::get<1>(one_v);
        double z_pre = std::get<2>(one_v);

        double x_post = std::get<0>(two_v);
        double y_post = std::get<1>(two_v);
        double z_post = std::get<2>(two_v);

        return sqrt(pow(x_pre-x_post, 2) + pow(y_post-y_pre, 2) + pow(z_post + z_pre, 2)) < SPEED_TRESH;
    
}



std::tuple<double, double, double > MobilePoint::CurrentPos(){

    float x = 0;
    float y = 0;
    float z = 0;



    cv::Mat threed_post = place_history_3d[place_history_3d.size()-1];

    x+= (double)threed_post.at<float>(0);
    y+= (double)threed_post.at<float>(1);
    z+= (double)threed_post.at<float>(2);
    

    return std::tuple<double, double, double> (x, y, z);


}


bool MobilePoint::WithinAMeter(cv::Mat thiso){

    double x_this = (double)thiso.at<float>(0);
    double y_this = (double)thiso.at<float>(0);
    double z_this = (double)thiso.at<float>(0);
    double x_that, y_that, z_that;

    
    std::tie(x_that, y_that, z_that) = this->CurrentPos();


    return sqrt(pow(x_this-x_that, 2) + pow(y_this-y_that, 2) + pow(z_this + z_that,2) )< 0.1;


}









bool MobilePoint::xlier_ratio(){
    double denum = (double)xlier_info.size();
    double num = 0;
    for (bool bools: xlier_info){
        if (bools) num++;


    }

    return num/denum > XLIER_RATIO_TRESH;
}

bool MobilePoint::NotGoodVel(MobilePoint* mb){

        

    vector<float > velocos_x; 
    vector<float > velocos_y; 
    vector<float > velocos_z; 


    for(int inner = 1; inner < (int) mb->place_history_3d.size(); ++inner){


        cv::Mat threed_pre = mb->place_history_3d[inner-1];
        cv::Mat threed_post = mb->place_history_3d[inner]; 

        double time_d = mb->time_history[inner] - mb->time_history[inner-1];

        float x_pre = threed_pre.at<float>(0);
        float y_pre = threed_pre.at<float>(1);
        float z_pre = threed_pre.at<float>(2);

        float x_post = threed_post.at<float>(0);
        float y_post = threed_post.at<float>(1);
        float z_post = threed_post.at<float>(2);


        velocos_x.push_back((x_post-x_pre)/time_d);
        velocos_y.push_back((y_post - y_pre)/time_d);
        velocos_z.push_back((z_post-z_pre)/time_d);

    }

    int total_count = (int) velocos_z.size();

    int   neg_count_x = 0;
    int   pos_count_x = 0;
    int   neg_count_y = 0;
    int   pos_count_y = 0;
    int   neg_count_z = 0;
    int   pos_count_z = 0;

    int fast_enough_x = 0; 
    int fast_enough_y = 0; 
    int fast_enough_z = 0; 


    for (int inner = 0; inner < (int)velocos_z.size(); ++inner ){

        float x_pre = velocos_x[inner];
        float y_pre = velocos_y[inner];
        float z_pre = velocos_z[inner];
        
        if(abs(x_pre) > 0.002 && abs(x_pre) < 0.1 ) fast_enough_x++;
        if(abs(y_pre) > 0.002 && abs(y_pre) < 0.1) fast_enough_y++;
        if(abs(z_pre) > 0.002) fast_enough_z++;

        x_pre > 0 ? pos_count_x++ : neg_count_x++;
        y_pre > 0 ? pos_count_y++ : neg_count_y++;
        z_pre > 0 ? pos_count_z++ : neg_count_z++;


    }


    double sr_x = double(neg_count_x)/double(pos_count_x);
    double sr_y = double(neg_count_y)/double(pos_count_y);
    double sr_z = double(neg_count_z)/double(pos_count_z);

    double fast_ratio_x = double(fast_enough_x)/double(total_count);
    double fast_ratio_y = double(fast_enough_y)/double(total_count);
    double fast_ratio_z = double(fast_enough_z)/double(total_count);


    // We check if there is too many changes in velocity (we don't want this)

    bool bx = sr_x<0.16 || sr_x > 0.84;
    bool by = sr_y<0.16 || sr_y > 0.84;
    bool bz = sr_z<0.16 || sr_z > 0.84;

    
    // We check if we are moving in any direction quickly enough

    bool fx = fast_ratio_x > 0.72;
    bool fy = fast_ratio_y > 0.72;
    bool fz = fast_ratio_z > 0.72;

    

    !((bx && by && bz) && (fx || fy )) ? cout << " GOODByE MY STATIONARY S STAIN\n" : cout << "=====MOBILE!! LETS GO.====\n";

    return  !((bx && by && bz) && (fx || fy ));

}



void MobilePoint::UpdateStates(std::vector<MobilePoint*>  & MobilePoints){

    cout << "Update states: size before: " << MobilePoints.size() << endl;


    auto mp = MobilePoints.begin();

    int xframe_count = 0;

    while(mp != MobilePoints.end()){

        switch ( (*mp)->dstate){

            case CANDIDATE : {
                (*mp)->looked_for++;

                if ((*mp)->looked_for > 6){


                    if((*mp)->seen < 4  || NotGoodVel(*mp)){
                        delete *mp;
                        mp = MobilePoints.erase(mp);
                    }

                    else {
                        cout << "SEEN! NEW FRAME " << endl;
                        (*mp)->dstate = XFRAME;
                        mp++;
                    }
                }


                mp++;
                break;

            }


            case XFRAME : {

                xframe_count++;
                (*mp)->looked_for++;
                mp++;
                break;
            }
        }

    }
    cout << "Update states: size after: " << MobilePoints.size() << endl;


    // get rid of stupid mobilepoints made
    mp = MobilePoints.begin();
    while(mp!=MobilePoints.end()){

        cv::Mat thiso = (*mp)->place_history_3d[0];
        auto thaso = (*mp)->CurrentPos();

        double x_this = (double)thiso.at<float>(0);
        double y_this = (double)thiso.at<float>(0);
        double z_this = (double)thiso.at<float>(0);



        double seperation = sqrt(pow(x_this-std::get<0>(thaso), 2) + pow(y_this-std::get<1>(thaso), 2) + pow(z_this - std::get<2>(thaso),2));


        // we are deleting old xframes that dont have an object. Bye bye pseudo pudge mudge. 
        if((*mp)->dstate == XFRAME && (*mp)->looked_for>13 && ((*mp)->object_index == -1 || (seperation< 0.005))) {
            delete *mp;
            mp = MobilePoints.erase(mp);

        }
        else{
            mp++;
        }
    }
    

    cout << "Update states: size after bafter: " << MobilePoints.size() << endl;

}



void MobilePoint::CreatePotential(cv::KeyPoint cand_kp, float depth){

    this->potential_finds.push_back(std::pair<cv::KeyPoint, float>(cand_kp, depth));


}

void MobilePoint::TestPotentials(ORB_SLAM3::Frame & cur_frame){
    //cout << "checking time frame\n";

    double t_delt = cur_frame.mTimeStamp;
    if(abs(this->time_history[time_history.size()-1]-t_delt)<0.05){
        this->potential_finds.clear();
        return;
    }

    auto dub = this->CalcV();

    //cout << "past checking time frame\n";

    for (auto p_f : this->potential_finds){
       // cout << "before proj\n";
        cv::Mat pos_3d = cur_frame.UnprojectStereo_MOB(p_f.first, p_f.second);

       // cout << "past proj\n";

        if (pos_3d.empty()) continue;
        double x  = (double)pos_3d.at<float>(0);
        double y = (double)pos_3d.at<float>(1);
        double z = (double)pos_3d.at<float>(2);

        double v_x, v_y, v_z;
        double cur_x,cur_y, cur_z;

        std::tie(v_x,v_y,v_z) = dub.first;
       // cout << "before current pos" << endl;
        std::tie(cur_x,cur_y,cur_z) = this->CurrentPos();
       // cout << "after current pos" << endl;

        double x_expected = v_x + cur_x;
        double y_expected = v_y + cur_y;
        double z_expected = v_z + cur_z;


        if(abs(x_expected-x) + abs(y_expected-y) + abs(z_expected -z) < 0.1){
           // cout <<"adding a fun one!";
            this->place_history_3d.push_back(pos_3d.clone());
            this->xlier_info.push_back(true);
            this->time_history.push_back(t_delt);
            this->seen++;
            this->potential_finds.clear();
            break;
        }
    }


    this->potential_finds.clear();
    return;

}


bool MobilePoint::InterviewXlier(cv::KeyPoint cand_kp, cv::Mat cand_desc,  MapPoint * cand_mp, std::vector<MobilePoint*>  & MobilePoints, double t_delt, bool outlier)  
    {
        if(!cand_mp){
            cout << "null map point " << endl;
            exit(1);
        }


        int bestDist = 256;
        int bestLevel = -1;
        int bestDist2 = 256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        int idx = 0;


        //cout << " lenght of mb is " << MobilePoints.size() << endl;
        // We find best match. Then we apply ratio test to see if match is anything to write home about 

        for (MobilePoint * mp : MobilePoints){

            if (!mp) cout << "oh oh null \n";

            const cv::Mat &d = mp->name_desc;

            //cout << "bef descp dist.....";
            const int dist = ORBmatcher::DescriptorDistance(cand_desc, d);
            //cout << "afeter descp dist \n";
            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = mp->name_keyp.octave;

                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = mp->name_keyp.octave;
                bestDist2 = dist;
            }

            ++idx;
            
        }


        // Apply ratio to second match (only if best and second are in the same scale level)
        if (!cand_mp) cout << "oh oh null cand \n";
        cv::Mat new_pos = cand_mp->GetWorldPos();


        if(bestDist<=28){



            if(bestLevel==bestLevel2 && bestDist>MFNNRATIO*bestDist2){

                if(!outlier) return false;


                // ratio test failed. You're looking like a potential new mobile object
                MobilePoint* mob_p = new MobilePoint(cand_kp, cand_desc, new_pos, t_delt);
                mob_p->dstate = CANDIDATE;
                MobilePoints.push_back(mob_p);
                return true;
            }
            if(bestLevel!=bestLevel2 || bestDist<=MFNNRATIO*bestDist2){


                // ratio test looking good. Mom would be proud. we update best candidate
                if(abs(MobilePoints[bestIdx]->time_history[MobilePoints[bestIdx]->time_history.size()-1]-t_delt)<0.05) return true;

                if(MobilePoints[bestIdx]->WithinAMeter(new_pos)){
                    MobilePoints[bestIdx]->place_history_3d.push_back(new_pos);
                    MobilePoints[bestIdx]->xlier_info.push_back(outlier);
                    MobilePoints[bestIdx]->time_history.push_back(t_delt);
                    MobilePoints[bestIdx]->seen++;
                    return false;

                }



            }
        }



        MobilePoint* mob_p = new MobilePoint(cand_kp, cand_desc, new_pos,t_delt);
        mob_p->dstate = CANDIDATE;
        MobilePoints.push_back(mob_p);
        return true;
    
            

    }
    



