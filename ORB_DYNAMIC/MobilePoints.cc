    
#include "MobilePoints.h"
#include "ORBextractor.h"
#include <vector>

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

    //std::vector<ORB_SLAM3::MapPoint*> 
    

// using namespace ORB_SLAM3::

using namespace ORB_SLAM3;
vector<MapPoint *> MobilePoints::SearchForMobile(ORB_SLAM3::Frame &CurrentFrame, const ORB_SLAM3::Frame &LastFrame, const float th, const bool bMono)

    {
        int nmatches = 0;



        cout << "starting\n";
        vector<MapPoint*> mobile_points;

        int HISTO_LEGTH_MOBILE = 50;

        bool mbCheckOrientation = true;

        // Rotation Histogram (to check rotation consistency)
        vector<int> rotHist[HISTO_LEGTH_MOBILE];
        for(int i=0;i<HISTO_LEGTH_MOBILE;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LEGTH_MOBILE;

        const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);

        const cv::Mat twc = -Rcw.t()*tcw;

        const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);

        const cv::Mat tlc = Rlw*twc+tlw;
        cout << "initial mats passed\n";

        const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
        const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;

        for(int i=0; i<LastFrame.N; i++)
        {
            MapPoint* pMP = LastFrame.mvpMapPoints[i];
            if(pMP)
            {
                cout << "iteration....begins!!\n";

                if(!LastFrame.mvbOutlier[i])
                {
                    // Project
                    cv::Mat x3Dw = pMP->GetWorldPos();
                    cv::Mat x3Dc = Rcw*x3Dw+tcw;

                    const float xc = x3Dc.at<float>(0);
                    const float yc = x3Dc.at<float>(1);
                    const float invzc = 1.0/x3Dc.at<float>(2);

                    if(invzc<0)
                        continue;

                    cv::Point2f uv = CurrentFrame.mpCamera->project(x3Dc);

                    if(uv.x<CurrentFrame.mnMinX || uv.x>CurrentFrame.mnMaxX)
                        continue;
                    if(uv.y<CurrentFrame.mnMinY || uv.y>CurrentFrame.mnMaxY)
                        continue;

                    int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave
                                                                                     : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

                    // Search in a window. Size depends on scale
                    float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                    vector<size_t> vIndices2;

                    if(bForward)
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv.x,uv.y, radius, nLastOctave);
                    else if(bBackward)
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv.x,uv.y, radius, 0, nLastOctave);
                    else
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv.x,uv.y, radius, nLastOctave-1, nLastOctave+1);

                    if(vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                    {
                        const size_t i2 = *vit;

                        if(CurrentFrame.mvpMapPoints[i2])
                            if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                                continue;

                        if(CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2]>0)
                        {
                            const float ur = uv.x - CurrentFrame.mbf*invzc;
                            const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                            if(er>radius)
                                continue;
                        }

                        const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                        const int dist = ORBmatcher::DescriptorDistance(dMP,d);

                        if(dist<bestDist)
                        {
                            bestDist=dist;
                            bestIdx2=i2;
                        }
                        
                        if ( dist > 150){
                            cout << "pushing back!!\n";


                            mobile_points.push_back(CurrentFrame.mvpMapPoints[i2]);
                            
                        }

                    }

                    if(bestDist<=ORBmatcher::TH_HIGH)
                    {
                        CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                        nmatches++;

                        if(mbCheckOrientation)
                        {
                            cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                                                        : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                                                : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                            cv::KeyPoint kpCF = (CurrentFrame.Nleft == -1) ? CurrentFrame.mvKeysUn[bestIdx2]
                                                                           : (bestIdx2 < CurrentFrame.Nleft) ? CurrentFrame.mvKeys[bestIdx2]
                                                                                                             : CurrentFrame.mvKeysRight[bestIdx2 - CurrentFrame.Nleft];
                            float rot = kpLF.angle-kpCF.angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LEGTH_MOBILE)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LEGTH_MOBILE);
                            rotHist[bin].push_back(bestIdx2);
                        }
                    }
                    if(CurrentFrame.Nleft != -1){
                        cv::Mat x3Dr = CurrentFrame.mTrl.colRange(0,3).rowRange(0,3) * x3Dc + CurrentFrame.mTrl.col(3);

                        cv::Point2f uv = CurrentFrame.mpCamera->project(x3Dr);

                        int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave
                                                                                         : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

                        // Search in a window. Size depends on scale
                        float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                        vector<size_t> vIndices2;

                        if(bForward)
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv.x,uv.y, radius, nLastOctave, -1,true);
                        else if(bBackward)
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv.x,uv.y, radius, 0, nLastOctave, true);
                        else
                            vIndices2 = CurrentFrame.GetFeaturesInArea(uv.x,uv.y, radius, nLastOctave-1, nLastOctave+1, true);

                        const cv::Mat dMP = pMP->GetDescriptor();

                        int bestDist = 256;
                        int bestIdx2 = -1;

                        for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                        {
                            const size_t i2 = *vit;
                            if(CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft])
                                if(CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft]->Observations()>0)
                                    continue;

                            const cv::Mat &d = CurrentFrame.mDescriptors.row(i2 + CurrentFrame.Nleft);

                            const int dist = ORBmatcher::DescriptorDistance(dMP,d);


                            if(dist<bestDist)
                            {
                                bestDist=dist;
                                bestIdx2=i2;
                            }
                        }

                        if(bestDist<=ORBmatcher::TH_HIGH)
                        {
                            CurrentFrame.mvpMapPoints[bestIdx2 + CurrentFrame.Nleft]=pMP;
                            nmatches++;
                            if(mbCheckOrientation)
                            {
                                cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                                                            : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                                                    : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                                cv::KeyPoint kpCF = CurrentFrame.mvKeysRight[bestIdx2];

                                float rot = kpLF.angle-kpCF.angle;
                                if(rot<0.0)
                                    rot+=360.0f;
                                int bin = round(rot*factor);
                                if(bin==HISTO_LEGTH_MOBILE)
                                    bin=0;
                                assert(bin>=0 && bin<HISTO_LEGTH_MOBILE);
                                rotHist[bin].push_back(bestIdx2  + CurrentFrame.Nleft);
                            }
                        }



                    }
                }
            }
        }

        //Apply rotation consistency
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ORB_SLAM3::ORBmatcher::ComputeThreeMaxima(rotHist,HISTO_LEGTH_MOBILE,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LEGTH_MOBILE; i++)
            {
                if(i!=ind1 && i!=ind2 && i!=ind3)
                {
                    for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    {
                        CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                        nmatches--;
                    }
                }
            }
        }

        return mobile_points;
    }


