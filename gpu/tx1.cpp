#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <pthread.h>
#include <iomanip>
#include <crow.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>
#include <Utils.hpp>

using namespace std;

#ifdef COMPILEDWITHC11
#define SET_CLOCK(t0) \
        std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();
#else
#define SET_CLOCK(t0) \
        std::chrono::monotonic_clock::time_point t0 = std::chrono::monotonic_clock::now();
#endif

#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())

std::string Vocab = "/home/ubuntu/ORB-SLAM2-GPU2016-final/Vocabulary/ORBvoc.txt";
std::string config = "/home/ubuntu/ORB-SLAM2-GPU2016-final/gpu/TX1.yaml";
std::string save_location = "./loose_pointcloud";

bool log_on = false;
std::string file_loc = "./";
std::ofstream logofs;

ORB_SLAM2::System* SLAM;

crow::SimpleApp app;
bool come_on_and_slam = false;
void *AppThread(void *threadid){
    app.port(18070).multithreaded().run();
}
int main(int argc, char **argv)
{
	bool viewer_on = false;
	//std::cout<< argc<< endl <<argv[1]<<endl;
	if(argc > 1){
		if(argv[1]){
			
			viewer_on = true;
		}
	}
	ORB_SLAM2::System SLAM_temp(Vocab,config,ORB_SLAM2::System::MONOCULAR,viewer_on);
	SLAM = &SLAM_temp;
    pthread_t threads[1];
    int rc;
    int i =0;
  
    CROW_ROUTE(app, "/")([](){
      return "Connection Check\n";
    });
    CROW_ROUTE(app, "/slam/start").methods("POST"_method)([](const crow::request& req){
      come_on_and_slam = true;
      return "come on and slam\n";
    });
    
    CROW_ROUTE(app, "/slam/reset")([](){
      SLAM->Reset();
      return "reset map\n";
    });
    CROW_ROUTE(app, "/slam/save").methods("POST"_method)([](const crow::request& req){
      SLAM->mpMapDrawer->SaveMapPoints(save_location);
      return "saving map\n";
    });
    CROW_ROUTE(app, "/slam/stop")([](){
      come_on_and_slam = false;
      return "saving map\n";
    });
    CROW_ROUTE(app, "/tracker/state").methods("POST"_method)([](const crow::request& req){
      if (SLAM->mpTracker->mState ==2){
        return "OK\n";
      }else if(SLAM->mpTracker->mState ==3){
        return "LOST\n";
      }else if(SLAM->mpTracker->mState ==-1){
        return "SYSTEM NOT READY\n";
      }else if(SLAM->mpTracker->mState ==0){
        return "NO IMAGES YET\n";
      }else if(SLAM->mpTracker->mState ==1){
        return "NOT INITIALIZED\n";
      }
      //return SLAM->mpTracker.mState;
    });
    CROW_ROUTE(app, "/pos_log/curr")([](){
      cout << endl << SLAM->mpTracker->mCurrentFrame.mTcw << endl;
      return "curr_pos\n";
    });
    CROW_ROUTE(app, "/pos_log/save").methods("POST"_method)([](const crow::request& req){
      auto x = crow::json::load(req.body);
      if (!x){
        return "please include a json object with the field of \"file_loc\"";
      }
      //this needs to begin the saving of log at the location specified in the json object
	  	//just start a log.
			log_on = true;
			file_loc = x["file_location"].s();
			save_location = file_loc + "/graph/keypoint_cloud";
			std::string slam_log_loc = file_loc + "/graph/slam_log.txt";
			logofs.open(slam_log_loc);
      cout << endl << x["file_location"].s() << endl;
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
	while(!come_on_and_slam){
		usleep(1000);
	}
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
      SLAM->TrackMonocular(im,tframe);
      POP_RANGE;
      SET_CLOCK(t2);

      double trackTime = TIME_DIFF(t2, t1);
      trackTimeSum += trackTime;
      tsum = tframe - tbuf[tpos];
      tbuf[tpos] = tframe;
      tpos = (tpos + 1) % 10;
	  	if(log_on){
				logofs << std::chrono::duration_cast<std::chrono::nanoseconds> (t1.time_since_epoch()).count()<< "\n";
				logofs << SLAM->mpTracker->mCurrentFrame.mTcw<<"\n";
			}

		
      //cerr << "Frame " << frameNumber << " : " << tframe << " " << trackTime << " " << 10 / tsum << "\n";
      ++frameNumber;
    }
		logofs.close();
    // Stop all threads
    SLAM->Shutdown();

    cerr << "Mean track time: " << trackTimeSum / frameNumber << " , mean fps: "  << "\n";

    return 0;
}
