/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>
#include <cassert>

#include <condition_variable>

#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "../../lcmtypes/mbot_imu_t.hpp"
#include "../../lcmtypes/mbot_image_t.hpp"
#include "../../lcmtypes/mbot_video_stream_t.hpp"
#include "../../lcmtypes/mbot_d_stream_t.hpp"

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

#define NUM_SAMPLES  40
#define IMUS_PER_IMG  30


using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

void lcm_imu_handler(void);

void lcm_photo_handler(void);



// Thread Synchronizers
std::mutex lcm_mutex;
std::condition_variable condv;
std::condition_variable condv_vid;

bool found_image;
bool found_depth;


// global values updated constantly

vector<ORB_SLAM3::IMU::Point> vImuMeas;

double vAccX, vAccY, vAccZ, vRotX, vRotY, vRotZ, vTime;
vector<vector<vector<int16_t>>> p_image(144, vector<vector<int16_t>>(192, vector<int16_t>(3)));
int64_t ni;




cv::Mat frame;
cv::Mat depth;


class Handler 
{
    public:
        ~Handler() {}
        void handleIMUMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const mbot_imu_t* msg)
        {
            std::lock_guard<std::mutex> lk(lcm_mutex);
            


            vAccX  =  msg->gyro[0];
            vAccY  =  msg->gyro[1];
            vAccZ  =  msg->gyro[2];
            vRotX  =  msg->mag[0];
            vRotY  =  msg->mag[1];
            vRotZ  =  msg->mag[2];
            vTime  =  msg->utime;

            vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAccX, vAccY, vAccZ, vRotX, vRotY, vRotZ, vTime));


            //keeps only latest IMUS_PER_IMG imu measurements in case there is some stalling from image lcm
            if(vImuMeas.size()>IMUS_PER_IMG){
                std::cout << "IMU vector full - discarding oldest" << std::endl;
                vImuMeas.erase(vImuMeas.begin());
            }

        }


        void handleDepth(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const mbot_d_stream_t* msg){
            std::lock_guard<std::mutex> lk(lcm_mutex);

            found_depth = true;
            cout << "In depth handler\n";
            depth = cv::imdecode(msg->image, cv::IMREAD_ANYDEPTH);


            cout << "Received Depth\n"; 


            condv.notify_all();

        }




        void handleStream(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const mbot_video_stream_t* msg){
            std::lock_guard<std::mutex> lk(lcm_mutex);
            
            found_image = true;
            ni++;


            cout << "In Video handler\n";
            frame = cv::imdecode(msg->image, cv::IMREAD_GRAYSCALE);
            // std::string frame_name = to_string(msg->timestamp);
            // cv::imwrite("./stream_data/"+frame_name+".jpg", frame);
            //imshow("Frame", frame);
            cout << "Received Image\n";


            condv.notify_all();
        }        


        void handleImageMessage(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan, 
            const mbot_image_t *msg)
            {
                std::lock_guard<std::mutex> lk(lcm_mutex);


                
                found_image = true;
                ni = msg->utime;
                std::cout << "\n======== IMAGE THREAD =========\n" <<std::endl;

                std::cout << "Reading image..." <<std::endl;

                for(int x = 0 ; x< 144; ++x){
                    for(int y= 0; y< 192; ++y){
                        for(int z = 0; z < 3; ++z)
                        {
                            p_image[x][y][z] = msg->encode_img[x][y][z];

                        }
                    }
                }

                std::cout << "Notifying ORBSLAM thread" <<std::endl;

                condv.notify_all();


                std::cout << "\n======== IMAGE THREAD END =========\n" <<std::endl;

            }



    
};


void lcm_imu_handler(void){

    // Makes lcm o
    lcm::LCM lcm;
    if(!lcm.good())
        return;
    Handler handlerObject;
    lcm.subscribe("MBOT_IMU", &Handler::handleIMUMessage, &handlerObject);



    while(0 == lcm.handle());

}

void lcm_video_handler(void){


    // Makes lcm o
    lcm::LCM lcm;
    if(!lcm.good())
        return;
    Handler handlerObject;
    lcm.subscribe("MBOT_VIDEO_STREAM", &Handler::handleStream, &handlerObject);
    lcm.subscribe("MBOT_D_STREAM", &Handler::handleDepth, &handlerObject);


    while(0 == lcm.handle());

}



void lcm_image_handler(void){

    // Makes lcm o
    lcm::LCM lcm;
    if(!lcm.good())
        return;
    Handler handlerObject;
    lcm.subscribe("MBOT_IMAGE", &Handler::handleImageMessage, &handlerObject);



    while(0 == lcm.handle());

}


void p_image_init(){

    vector<int16_t> one_d( 3, 0);
    for(int x = 0 ; x< 144; ++x){
        vector<vector<int16_t> >two_d;
        for(int y= 0; y< 192; ++y){
            two_d.push_back(one_d);
            
        }

        p_image.push_back(two_d);
    }


}




double ttrack_tot = 0;
int main(int argc, char *argv[])
{

    ni = 0;

    if(argc < 7)
    {
        cerr << endl << "Usage: ./mono_inertial_a3\n  path_to_vocabulary\n  path_to_settings\n  path_to_sequence_folder_1\n  path_to_times_file_1\n  (path_to_image_folder_2\n    path_to_times_file_2 ...\n   path_to_image_folder_N path_to_times_file_N)\n  DO_IMU " << endl;
        return 1;
    }

    bool DO_IMU = atoi(argv[argc-1]);


    const int num_seq = (argc-3)/2;
    assert(num_seq == 0); // Just to make sure the command args are correct -Bryce
    
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-4) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-2]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    

    

    int tot_images = 0;
    


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(8);


    std::cout << "\ngoind to sleep befreo" << std::endl;



    //Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(
                        argv[1],
                        argv[2],
                        ORB_SLAM3::System::RGBD,
                        true);


    //std::thread lcm_imu_thread; 
    if (DO_IMU){

        //lcm_imu_thread = std::thread(lcm_imu_handler);
    }


    std::thread lcm_image_thread(lcm_video_handler);



    while(1)
    {



        // we wait for next image to be handled
        std::unique_lock<std::mutex> lk(lcm_mutex);
        std::cout << "\ngoind to sleep" << std::endl;
        condv.wait(lk, []{return found_image && found_depth;});

        std::cout << "\n======== ORBSLAM THREAD =========\n" <<std::endl;

        found_image = false;
        found_depth = false;




        // Main loop

        vector<int> sizes = {144,192,3};
        cv::Mat im(sizes, CV_16U);



        

        //double tframe = vTimestampsCam[seq][ni];
        double tframe = ni * 0.1;
        
        #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif
        
        std::cout << "Calling 'Trackrgbd'"<< std::endl;

        SLAM.TrackRGBD(frame,depth,tframe);

        
        std::cout << "-----------f" << std::endl;
            

        #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        ttrack_tot += ttrack;
        

        std::cout << "-----------g" << std::endl;

        std::cout << "-----------h" << std::endl;

        // if(seq < num_seq - 1)
        // {
        //     cout << "Changing the dataset" << endl;

        //     //SLAM.ChangeDataset();
        // }

        std::cout << "Clearing IMU vector" << std::endl;

        //vImuMeas.clear();
        std::cout << seq << std::endl;
        std::cout << "\n======== ORBSLAM THREAD END =========\n" <<std::endl;


    }


    //lcm_imu_thread.join();
    lcm_image_thread.join();

    std::cout << "the end" << endl;

    SLAM.Shutdown();
    return 0;
}

/*
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(10000);
    vstrImages.reserve(10000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(10000);
    vAcc.reserve(10000);
    vGyro.reserve(10000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
*/
