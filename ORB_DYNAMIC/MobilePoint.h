
#ifndef MOBILE_POINT_H
#define MOBILE_POINT_H


#include <utility>
#include <tuple>
#include "Frame.h"
#include "MapPoint.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#define LOOKED_FOR_THRESH 9
#define SEEN_THRESH       6
#define XLIER_RATIO_TRESH 0.5
#define TH_HIGH_MOB       24


#define SPEED_TRESH       0.1
#define DIST_THRESH       0.02


#define MFNNRATIO         0.6



class MobilePoint{
    public:
    // Constructor
    MobilePoint (const cv::KeyPoint kp, const cv::Mat desc, const cv::Mat place, double t_delt );


    // Returns average velocity and variance
    std::pair<std::tuple<double, double, double>, double> CalcV( int starty = 0);


    // Tests if state XFRAME is legitness.
    static bool NotGoodVel(MobilePoint* mb);


    // Gives mobilepoint current position
    std::tuple<double, double, double > CurrentPos();



    // Updates states of atlase's thing
    static void UpdateStates(std::vector<MobilePoint*>  & MobilePoints);

    // Debug print function
    static void PrintHiredInfo(std::vector<MobilePoint*> & MobilePoints);


    // Make from orbmatcher
    void CreatePotential(cv::KeyPoint cand_kp, float depth);

    // Test these
    void TestPotentials(ORB_SLAM3::Frame & cur_frame);



    // Main function. Takes in an outlier from tracking, and interviews it. Either finds it original, or a match of a  current Mobilepoint.
    static bool InterviewXlier(cv::KeyPoint c_kp, cv::Mat cand_desc,  ORB_SLAM3::MapPoint * cand_mp, std::vector<MobilePoint*>  & MobilePoints, double t_delt, bool outlier);  


    bool WithinAMeter(cv::Mat thiso);

    bool SimilarVelocities(MobilePoint *mb);


    // Checks if we have enough outliers
    bool xlier_ratio();


    public:

    enum  dynamic_state
    {
        CANDIDATE, // still sus.
        XFRAME    // Crewmate. don't care about frame.

    };

    // necessary for determining what to do with this mobilepoint
    dynamic_state dstate;

    // Mobile points identifiers. 
    cv::Mat name_desc;
    cv::KeyPoint name_keyp;
    int object_index;            //assigned once mobilepoint is set as XFRAME
    

    // history of where this mobile point has been seen 
    // vector<ORB_SLAM3::MapPoint*> place_history;
    vector<cv::Mat> place_history_3d; 

    // parallel to place_history. "xlier_info[i] == true" means place_history[i]  is an outlier, False means inliner
    vector<bool> xlier_info;

    // parallel to place history. "time_history[i]" identifies the time in seconds in which this position was recorded
    vector<double> time_history;

    vector<std::pair<cv::KeyPoint, float>> potential_finds;
    
    // variables used when our mobilepoint is a candidate. If the candidate has been seen in multiple frames, 
    // at a certain threshold, it will be adapted to our belief system. 
    int looked_for;
    int seen;



};


#endif 