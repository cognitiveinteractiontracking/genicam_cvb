#include <opencv2/opencv.hpp>
#include <genicam/genicam.hpp>
#include <chrono>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <camera_info_manager/camera_info_manager.h>
// #include <cv_bridge/cv_bridge.h>

// Image from the camera
// It is filled by Stemmer Camera API
static cv::Mat frame, frameDstTmp, *bgrFrame, *frameDst, frameDstRectified;
static const size_t DRIVERPATHSIZE = 256;
// Camera object of Stemmer Camera API
static IMG hCamera = NULL;
long IMGheight;
long IMGwidth;
static int colorChannelsSrc = 1;
static int colorChannelsDst = 3;
static int monoAsBayer = 0;

// Argument Handler
//static ArgHandler argHandler;

// Gui Options
static int gui = false;
static int printFps = false;

// Camera parameter
static std::string cameraParamterPrefix;
static std::vector<std::pair<std::string, std::string>> cameraParamter;
static std::string rosCalbirationFile;
static int triggerMode;
static int ptpTimeSync;

// ROS send parameters
static std::string topic;
static std::string frame_id;
static int streamCounterRst = 1; // Send ever n'th image
static int streamCounter = 1;
static int expectedFps = 48;
ros::Publisher imagePublisher;
ros::Publisher cameraInfoPublisher;
sensor_msgs::Image msgImage;
sensor_msgs::CameraInfo msgCameraInfo;
cv::Mat img;

static std::string driverpath = "";
const std::string cvbDriverPath("/etc/opt/cvb/drivers/GenICam.ini");

// FPS calculation
int fpsCounter = 0;
int fpsStreamCounter = 0;
static std::chrono::high_resolution_clock::time_point timeStamp;

/**
 * Prototypes
 */

// After configuration is done in main, this function loops for ever
int processEverything(void);
// Handle the image
int process(void);
// Get the time difference in seconds
//inline float getTimeDiff(const clock_t &start, const clock_t &end);
inline float getTimeDiff(const std::chrono::high_resolution_clock::time_point &start,
                         const std::chrono::high_resolution_clock::time_point &end);
// Handle program options
void programOptions(ros::NodeHandle &n);

void loadCamera() {
  int set_cam_params = 0;
  int attempt_thresh = 5;
  while (true) {
    // Get Driver from argument
    char driverPath[DRIVERPATHSIZE] = {0};
    TranslateFileName("%CVB%/drivers/GenICam.vin", driverPath, DRIVERPATHSIZE);

    // Load the camera via Stemmer API
    ROS_INFO_STREAM("Load the camera driver from " << driverPath);
    cvbbool_t success = LoadImageFile(driverPath, hCamera);
    if (!success) {
      ROS_ERROR_STREAM("Error loading " << driverPath << " driver!");
      ros::shutdown();
      exit(-1);
    }
    ROS_INFO_STREAM("Load " << driverPath << " successful.");
    // access camera config
    if (!genicam::set_camera_parameter(cameraParamter, hCamera)) {
      set_cam_params++;
      ROS_WARN_STREAM("+++ Camera access Error! +++ with tried attempts:" << set_cam_params);
      ReleaseObject(hCamera);
      usleep(1e6);
      if (set_cam_params == attempt_thresh) {
        ROS_WARN("Attempts reached treshold: %d. Shutdown Node.", attempt_thresh);
        ros::shutdown();
        exit(-1);
      }
    } else {
      break;
    }
  }
}

int process() {
  // Getting the address of the camera source image
  void *lpBaseAddress = NULL;
  intptr_t lXInc;
  intptr_t lYInc;
  const cvbbool_t status = GetLinearAccess(hCamera, 0, &lpBaseAddress, &lXInc, &lYInc);
  if (!status) {
    ROS_INFO_STREAM("Error GetLinearAccess");
    return -1;
  }

  ROS_DEBUG_STREAM("(lXInc, lYInc): " << lXInc << " " << lYInc);
  // Copy the frame out of the ring buffer
  if (lpBaseAddress != 0) {
    memcpy(&frame.at<char>(0), (char*) lpBaseAddress, IMGwidth * IMGheight * colorChannelsSrc);
    // Color conversion
    if (colorChannelsSrc == 1 && colorChannelsSrc != colorChannelsDst && colorChannelsDst == 3) {
      cv::cvtColor(frame, frameDstTmp, CV_BayerGB2BGR);
      frameDst = &frameDstTmp;
    } else if (colorChannelsDst == colorChannelsSrc && colorChannelsDst == 3) {
      frameDst = &frame;
    } else if (colorChannelsDst == colorChannelsSrc && colorChannelsDst == 1) {
      if (monoAsBayer) {
        frameDst = &frame;
      } else {
        cv::cvtColor(frame, frameDstTmp, CV_BayerGB2GRAY);
        frameDst = &frameDstTmp;
      }
    } else {
        ROS_WARN_STREAM("Unsupported number of color channels '" << colorChannelsDst << "'for ROS image");
    }
    if (streamCounter >= streamCounterRst) {
      ++fpsStreamCounter;

      if (frameDst->channels() == 3) {
          msgImage.encoding = sensor_msgs::image_encodings::RGB8;
      } else { // (frameDst->channels() == 3)
//          msgImage.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
          msgImage.encoding = sensor_msgs::image_encodings::MONO8;
      }

      msgImage.height = IMGheight;
      msgImage.width = IMGwidth;
      msgImage.is_bigendian = false;

      double camTimestamp;
      G2GetGrabStatus(hCamera, GRAB_INFO_TIMESTAMP, camTimestamp);
      ROS_INFO_STREAM("GRAB_INFO_TIMESTAMP: " << std::fixed << camTimestamp);
      ROS_INFO_STREAM("GRAB_INFO_TIMESTAMP/1000000000: " << std::fixed << camTimestamp/1000000000.0);
      ROS_INFO_STREAM("GRAB_INFO_TIMESTAMP/2^30: " << std::fixed <<       camTimestamp/1073741824.0);
      ROS_INFO_STREAM("GRAB_INFO_TIMESTAMP/2^31: " << std::fixed <<       camTimestamp/2147483648.0);
      ROS_INFO_STREAM("GRAB_INFO_TIMESTAMP/2^32: " << std::fixed <<       camTimestamp/4294967296.0);
      long long camTimestamp_long = (long long)(camTimestamp);
      ROS_INFO_STREAM("GRAB_INFO_TIMESTAMP: " << camTimestamp_long);
      msgImage.header.stamp = ros::Time(uint32_t(camTimestamp_long >> 32), uint32_t((camTimestamp_long << 32) >> 32));
//        time_t secondsSinceEpoch = floor(long int(camTimestamp));
//        boost::posix_time::ptime boost_time = boost::posix_time::from_time_t(secondsSinceEpoch);
//        ros::Time::fromNSec(camTimestamp);
//        ros::Time time = ros::Time(uint32_t(camTimestamp >> 32), uint32_t((camTimestamp << 32) >> 32));
//        msgImage.header.stamp = time;
      ROS_INFO_STREAM("Cam Time in Ros:" << msgImage.header.stamp << "\t Ros Time now:" << ros::Time::now());
      ROS_INFO_STREAM("Difference:" << std::fixed << (ros::Time::now().toSec() - camTimestamp/1000000000.0));
      ROS_INFO_STREAM("Difference (2^30):" << std::fixed << (ros::Time::now().toSec() - camTimestamp/1073741824.0));
      ROS_INFO_STREAM("Difference (2^31):" << std::fixed << (ros::Time::now().toSec() - camTimestamp/2147483648.0));
      ROS_INFO_STREAM("Difference (2^32):" << std::fixed << (ros::Time::now().toSec() - camTimestamp/4294967296.0));
      ROS_INFO_STREAM("\n");
      if(ptpTimeSync) {
        // TODO Timestamp sending
      } else {
        msgImage.header.stamp = ros::Time::now();
        // TODO DELETE
        //msgImage.header.stamp = ros::Time(camTimestamp/1000000000.0);
      }
      msgCameraInfo.header.stamp = msgImage.header.stamp;
      msgImage.data.resize(IMGheight * IMGwidth * frameDst->channels());
      msgImage.step = IMGwidth * frameDst->channels();
//      for (int idx = 0; idx < IMGwidth * IMGheight; ++idx) {
//          const int xIdx = idx / IMGwidth;
//          const int yIdx = idx % IMGwidth;
//          memcpy(msgImage.data.data() + (yIdx * IMGwidth + xIdx), frameDst->data + idx, 1);
//      }
      memcpy(msgImage.data.data(), frameDst->data, IMGheight * IMGwidth * frameDst->channels());
      imagePublisher.publish(msgImage);
      cameraInfoPublisher.publish(msgCameraInfo);

      if (gui) {
        cv::imshow("Camera Output", frame);
        cv::waitKey(1);
      }

      streamCounter = 0;
    } else {
      // Don't do anything
    }
    ++streamCounter;
    ++fpsCounter;
  } else {
    ROS_INFO_STREAM("Error BaseAddress is ZERO");
    return -1;
  }

  if (printFps) {
    float timeDiff;
    if ((timeDiff = getTimeDiff(timeStamp, std::chrono::high_resolution_clock::now())) > 1.0f) {
      ROS_INFO_STREAM("FPS to gstreamer: " << fpsStreamCounter / timeDiff);
      ROS_INFO_STREAM("FPS from camera: " << fpsCounter / timeDiff);
      fpsCounter = 0;
      fpsStreamCounter = 0;
      timeStamp = std::chrono::high_resolution_clock::now();
    }
  }

  return 0;
}

int processEverything(void) {
  // Wait for next image to be acquired
  // (returns immediately if unprocessed image are in the ring buffer)
  cvbres_t result = G2Wait(hCamera);

//  double ans1;
//  G2GetGrabStatus(hCamera, GRAB_INFO_TIMESTAMP, ans1);
//  ROS_INFO_STREAM("GRAB_INFO_TIMESTAMP: " << ans1);

  if (result < 0) {
    ROS_INFO_STREAM("Error with G2Wait");
    double ans;
    G2GetGrabStatus(hCamera, GRAB_INFO_GRAB_ACTIVE, ans);
    ROS_INFO_STREAM("GRAB_INFO_GRAB_ACTIVE: " << ans);
    if(triggerMode)
      ROS_WARN("TriggerMode is active. Wait for trigger.");
//    while(ans == 0) {
//      genicam::access_bool("AcquisitionStop", true, hCamera);
//      ROS_INFO("AcquisitionStop");
//      usleep(500000);
//      genicam::access_bool("AcquisitionStart", true, hCamera);
//      ROS_INFO("AcquisitionStart");
//      usleep(500000);
//      G2GetGrabStatus(hCamera, GRAB_INFO_GRAB_ACTIVE, ans);
//      ROS_INFO_STREAM("GRAB_INFO_GRAB_ACTIVE: " << ans);
//    }
    G2GetGrabStatus(hCamera, G2INFO_NumBuffersPending, ans);
    ROS_INFO_STREAM("G2INFO_NumBuffersPending: " << ans);
    G2GetGrabStatus(hCamera, G2INFO_NumBuffersLost, ans);
    ROS_INFO_STREAM("G2INFO_NumBuffersLost: " << ans);
    G2GetGrabStatus(hCamera, G2INFO_NumBuffersCorrupt, ans);
    ROS_INFO_STREAM("G2INFO_NumBuffersCorrupt: " << ans);
    if(ans > 0) {
      ROS_WARN_STREAM("G2INFO_NumBuffersCorrupt: " << ans << ". Shutdown node.");
      // stop the grab (kill = true: wait for ongoing frame acquisition to stop)
      G2Freeze(hCamera, true);
      // free camera
      ReleaseObject(hCamera);

      ros::shutdown();
      exit(-1);
    }
    // Try to stop and start the grabbing again
    G2Freeze(hCamera, true);
    G2Grab(hCamera);
  }

  // Grab the image and copy it to the variable frame
  return process();

}

inline float getTimeDiff(const std::chrono::high_resolution_clock::time_point &start,
                         const std::chrono::high_resolution_clock::time_point  &end) {
  const auto elapsed = end - start;
  const float seconds = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() / 1e3;
  return seconds;
}

void programOptions(ros::NodeHandle &n) {

  n.param<int>("stream_counter_reset", streamCounterRst, 0); // Process every n'th image
  n.param<std::string>("driverpath", driverpath, "/tmp/"); // Path of the driver location (*.ini)
  n.param<int>("print_fps", printFps, 0); // Print the FPS every second
  n.param<int>("color_channels_src", colorChannelsSrc, 1); // Number of channels in the image (Needed by OpenCV)
  n.param<int>("color_channels_dst", colorChannelsDst, 3); // Number of channels to send via ROS
  n.param<int>("mono_as_bayer", monoAsBayer, 0); // Sends mono images as bayer pattern
  n.param<int>("gui", gui, 0); // Show the rectified image by OpenCV
  n.param<int>("expected_fps", expectedFps, 1); // Expected FPS (Needed by gstreamer)
  n.param<std::string>("topic", topic, ""); // Topic to send the images
  n.param<std::string>("frame", frame_id, ""); // Frame id of the camera
  n.param<std::string>("camera_paramter_prefix", cameraParamterPrefix, "config"); // JSON Object with camera parameter
  n.param<std::string>("ros_calibration", rosCalbirationFile, ""); // Path to ros calibration.yaml file
  n.param<int>("trigger_mode", triggerMode, 1); // TriggerMode
  n.param<int>("ptp_time_sync", ptpTimeSync, 0); // use ptp to time synchronization

  // Load the parameter for the camera
  // Get the full namespace for the camera config
  const std::string nsParameter = n.getNamespace() + std::string("/") + cameraParamterPrefix + std::string("/");
  ROS_INFO_STREAM( "Explore namespace '" << nsParameter << "' for camera parameters");
  // Process all parameters on the parameter server
  std::vector<std::string> keys;
  n.getParamNames(keys);
  for(auto it = keys.begin(); it != keys.end(); ++it) {
    if (!it->substr(0, nsParameter.size()).compare(nsParameter)) {
        std::string keyParam = *it;
        std::string key = it->substr(nsParameter.size(), it->size()-1);
        std::string value;
        n.getParam(keyParam, value);

        cameraParamter.push_back(std::make_pair(key, value));
        ROS_INFO_STREAM( "Load : " << key << " with value " << value);
    }
  }
  cameraParamter.push_back(std::make_pair("AcquisitionFrameRate", to_string(expectedFps)));
  if(triggerMode) {
    cameraParamter.push_back(std::make_pair("TriggerSelector", "2"));
    cameraParamter.push_back(std::make_pair("TriggerMode", "1"));
    cameraParamter.push_back(std::make_pair("TriggerActivation", "0"));
    cameraParamter.push_back(std::make_pair("TriggerSource", "21")); // 21 for Line5 - In (Jai Spark)
  } else {
    cameraParamter.push_back(std::make_pair("TriggerMode", "0"));
  }
  if(ptpTimeSync) {
    cameraParamter.push_back(std::make_pair("GevIEEE1588", "1"));
  } else {
    cameraParamter.push_back(std::make_pair("GevIEEE1588", "0"));
  }

  msgImage.header.frame_id = frame_id;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "genicam_node");
  ros::NodeHandle n("~");
  programOptions(n);

  if (!monoAsBayer) {
    imagePublisher = n.advertise<sensor_msgs::Image>(topic, 2);
  } else if (monoAsBayer && colorChannelsSrc == 1 && colorChannelsDst == 1 ) {
    imagePublisher = n.advertise<sensor_msgs::Image>(topic/* + std::string("/bayer")*/, 2);
  } else {
    ROS_ERROR("(mono_as_bayer && color_channels_src == 1 && color_channels_dst == 1) || !monoAsBayer");
    return 1;
  }
  cameraInfoPublisher = n.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

  // Copy the driver
  ROS_INFO_STREAM("Copy " << driverpath << " to " << cvbDriverPath);
  std::ifstream source(driverpath, std::ios::binary);
  std::ofstream dest(cvbDriverPath, std::ios::binary);
  dest << source.rdbuf();
  source.close();
  dest.close();

  loadCamera();

  // Height & Width of the camera image
  IMGheight = ImageHeight(hCamera);
  IMGwidth = ImageWidth(hCamera);

  // Allocation of the framedata
  frame.create(IMGheight, IMGwidth, CV_8UC(colorChannelsSrc));
  frameDstTmp.create(IMGheight, IMGwidth, CV_8UC(colorChannelsDst));

  cvbres_t result = G2Grab(hCamera);
  if (result < 0) {
    ROS_ERROR_STREAM("Error starting G2Grab");
  } else {
      ROS_INFO_STREAM("Successfully started G2Grab");
  }

  ROS_INFO_STREAM("start grabbing");

  // Get the start clock once
  timeStamp = std::chrono::high_resolution_clock::now();

  //fill camera_info
  if(!rosCalbirationFile.empty()) {
    camera_info_manager::CameraInfoManager cameraInfoManager(n, ros::this_node::getName(), rosCalbirationFile);
    msgCameraInfo = cameraInfoManager.getCameraInfo();
  }
  msgCameraInfo.header.frame_id = frame_id;
  msgCameraInfo.height = (uint) IMGheight;
  msgCameraInfo.width = (uint) IMGwidth;
  msgCameraInfo.binning_x = genicam::read_int("BinningHorizontal", hCamera);
  msgCameraInfo.binning_y = genicam::read_int("BinningVertical", hCamera);
//  msgCameraInfo.roi.width = genicam::read_int("Width", hCamera);
//  msgCameraInfo.roi.height = genicam::read_int("Height", hCamera);
//  msgCameraInfo.roi.x_offset = genicam::read_int("OffsetX", hCamera);
//  msgCameraInfo.roi.y_offset = genicam::read_int("OffsetY", hCamera);

//  ROS_INFO_STREAM(genicam::get_all_camera_properties());

  while (true) {
    if (processEverything() != 0) {
      break;
    }
  }

  ROS_INFO_STREAM("Stop acquisition");

  // stop the grab (kill = true: wait for ongoing frame acquisition to stop)
  result = G2Freeze(hCamera, true);

  // free camera
  ReleaseObject(hCamera);

  // this state should actually never be reached, so we return an error code
  return -1;
}


