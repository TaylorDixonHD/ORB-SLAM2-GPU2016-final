#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <pthread.h>
#include <iomanip>
#include <crow.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>
#include <Utils.hpp>

using namespace std;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#else
#define SET_CLOCK(t0) \
        std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif

#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

std::string Vocab = "../Vocabulary/ORBvoc.txt";
std::string config = "./TX1.yaml";
std::string save_location = "./loose_pointcloud";
ORB_SLAM2::System SLAM(Vocab,config,ORB_SLAM2::System::MONOCULAR,true);

crow::SimpleApp app;
bool come_on_and_slam = true;
void *AppThread(void *threadid){
    app.port(18070).multithreaded().run();
}
int main(int argc, char **argv)
{

    pthread_t threads[1];
    int rc;
    int i =0;
  
    CROW_ROUTE(app, "/")([](){
      return "Connection Check\n";
    });
    CROW_ROUTE(app, "/start_slam")([](){
      come_on_and_slam = true;
      return "come on and slam\n";
    });
    //set the save location with /set_save_loc/"save_here"
    CROW_ROUTE(app, "/set_save_loc/<string>")([](string save_loc){
      save_location = save_loc;
      return "save location set\n";
    });
    CROW_ROUTE(app, "/reset_slam")([](){
      SLAM.Reset();
      return "reset map\n";
    });
    CROW_ROUTE(app, "/save_slam")([](){
      SLAM.mpMapDrawer->SaveMapPoints(save_location);
      return "saving map\n";
    });
    CROW_ROUTE(app, "/stop_slam")([](){
      come_on_and_slam = false;
      return "saving map\n";
    });
    CROW_ROUTE(app, "/tracker_state")([](){
      if (SLAM.mpTracker->mState ==2){
        return "OK\n";
      }else if(SLAM.mpTracker->mState ==3){
        return "LOST\n";
      }else if(SLAM.mpTracker->mState ==-1){
        return "SYSTEM NOT READY\n";
      }else if(SLAM.mpTracker->mState ==0){
        return "NO IMAGES YET\n";
      }else if(SLAM.mpTracker->mState ==1){
        return "NOT INITIALIZED\n";
      }
      //return SLAM->mpTracker.mState;
    });
    CROW_ROUTE(app, "/curr_pos")([](){
      cout << endl << SLAM.mpTracker->mCurrentFrame.mTcw << endl;
      return "curr_pos\n";
    });
    CROW_ROUTE(app, "/save_pos").methods("POST"_method)([](){
      auto x = crow::json::load(req.body);
      if (!x){
        return crow::response(400);
      }
      
      cout << endl << x["file_loc"] << endl;
      return "save_pos\n";
    });

    rc = pthread_create(&threads[0], NULL, AppThread, (void *)i);
    /*if(argc != 4) {
        cerr << endl << "Usage: " << argv[0] << " [path to vocabulary] [path to settings] [seconds to run]" << endl;
        return 1;
    }*/

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //find declaration in /include/system.h
    

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // const char * gst = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)I420, framerate=(fraction)24/1 ! nvvidconv flip-method=2 ! videoconvert ! appsink";
    const char * gst = "v4l2src device=/dev/video0 ! video/x-raw, width=1280, height=720, framerate=30/1 ! videoconvert ! appsink";
    cv::VideoCapture cap(gst);
    if (!cap.isOpened()) {
      printf("can not open camera or video file\n%s", gst);
      return -1;
    }

    double tsum = 0;
    double tbuf[10] = {0.0};
    int tpos = 0;
    double trackTimeSum = 0.0;
    // Main loop
    cv::Mat im;
    SET_CLOCK(t0);
    int frameNumber = 0;
    while (come_on_and_slam) {
      //cout << "Start processing sequence ..." << endl;
      cap >> im;
      if (im.empty()) continue;
      SET_CLOCK(t1);
      double tframe = TIME_DIFF(t1, t0);
      

      PUSH_RANGE("Track image", 4);
      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im,tframe);
      POP_RANGE;
      SET_CLOCK(t2);

      double trackTime = TIME_DIFF(t2, t1);
      trackTimeSum += trackTime;
      tsum = tframe - tbuf[tpos];
      tbuf[tpos] = tframe;
      tpos = (tpos + 1) % 10;
      //cerr << "Frame " << frameNumber << " : " << tframe << " " << trackTime << " " << 10 / tsum << "\n";
      ++frameNumber;
    }
    // Stop all threads
    SLAM.Shutdown();

    cerr << "Mean track time: " << trackTimeSum / frameNumber << " , mean fps: "  << "\n";

    return 0;
}
