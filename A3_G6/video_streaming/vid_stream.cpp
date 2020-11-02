#include <opencv2/opencv.hpp>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/mbot_video_stream_t.hpp"

using namespace cv;
using namespace std;


class Handler
{
    public:
        ~Handler() {}

        void handleStream(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const mbot_video_stream_t* msg){
            cout << "In handler\n";
            Mat frame = imdecode(msg->image, IMREAD_GRAYSCALE);
            String frame_name = to_string(msg->timestamp);
            imwrite("./stream_data/"+frame_name+".jpg", frame);
            //imshow("Frame", frame);
            cout << "Received Image\n";
        }
};



int main(){
    lcm::LCM lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    if(!lcm.good()){
        return 1;
    }
    Handler handlerObject;
    lcm.subscribe("MBOT_VIDEO_STREAM", &Handler::handleStream, &handlerObject);
    cout << "subscribed\n";
    while(true){
        lcm.handle();
        cout << "handling\n";
    }
    return 0;
}
