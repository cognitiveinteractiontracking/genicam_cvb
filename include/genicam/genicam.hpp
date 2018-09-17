#ifndef _GENICAM_H_
#define _GENICAM_H_

#include <string>
#include <arghandler/arghandler.h>

// Stemmer Imaging API
#include <iCVCDriver.h>
#include <iCVCImg.h>
#include <iCVGenApi.h>
#include <CVCError.h>
#include <iCVCUtilities.h>


namespace genicam {
  // reads a feature of type T via CVGenApi
  template<typename T>
  void read(const std::string nodeName, const T value, IMG hCamera);
  int read_int(const std::string nodeName, IMG hCamera);
  double read_double(const std::string nodeName, IMG hCamera);
  void read_bool(const std::string nodeName, IMG hCamera);
  void read_str(const std::string nodeName, IMG Camera);
  template<typename T>
  bool access( std::string nodeName, T value, IMG hCamera);
  bool access_int( std::string nodeName, int value, IMG hCamera);
  bool access_double( std::string nodeName, double value, IMG hCamera);
  bool access_bool( std::string nodeName, bool value, IMG hCamera);
  bool access_str( std::string nodeName, std::string value, IMG hCamera);
  bool set_camera_parameter(const std::vector<std::pair<std::string, std::string>> &cameraParameter, IMG hCamera);
}

#endif
