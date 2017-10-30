#include <genicam/genicam.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <math.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iterator>

// Stemmer Imaging API
#include <iCVCDriver.h>
#include <iCVCImg.h>
#include <iCVGenApi.h>
#include <CVCError.h>
#include <iCVCUtilities.h>

using namespace std;

namespace genicam {

// reads a feature of type T via CVGenApi
template<typename T>
void read(const std::string nodeName, const T value, IMG hCamera)
{
  //cout << "Get "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {
      if (typeid(value) == typeid(double)) {
        double val = 0.0;
        result = NGetAsFloat(hNode, val);
        //cout << val << " (double)" << endl;
      } else if (typeid(value) == typeid(cvbbool_t)) {
        cvbbool_t val = false;
        result = NGetAsBoolean(hNode, val);
        //cout << val << " (bool)" << endl;
      } else if (typeid(value) == typeid(cvbint64_t)) {
        cvbint64_t val = 0;
        result = NGetAsInteger(hNode, val);
        //cout << val << " (int)" << endl;
      } else if (typeid(value) == typeid(string)) {
        char* val;
        size_t len;
        result = NGetAsString(hNode, val, len);
        try {
          //cout << string(val,len) << " (string)" << endl;
        } catch (std::logic_error ex) {
          cout << "error at accessing string " << nodeName << endl;
        }
      } else
        result = -1;

      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
  }
}


int read_int(const std::string nodeName, IMG hCamera)
{
  cout << "Get "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {
      cvbint64_t val = 0;
      result = NGetAsInteger(hNode, val);
      cout << val << " (int)" << endl;
      return val;


      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
  }
  return -1;
}

double read_double(const std::string nodeName, IMG hCamera)
{
  cout << "Get "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {
      double val = 0.0;
      result = NGetAsFloat(hNode, val);
      cout << val << " (double)" << endl;
      return val;


      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
  }
}

void read_bool(const std::string nodeName, IMG hCamera)
{
  cout << "Get "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {
      cvbbool_t val = false;
      result = NGetAsBoolean(hNode, val);
      cout << val << " (bool)" << endl;


      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
  }
}

void read_str(const std::string nodeName, IMG hCamera)
{
  cout << "Get "<< nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {
      char* val;
      size_t len;
      result = NGetAsString(hNode, val, len);
      try {
        cout << string(val,len) << " (string)" << endl;
      } catch (std::logic_error ex) {
        cout << "error at accessing string " << nodeName << endl;
      }


      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
  }
}
// access a feature via CVGenApi
template<typename T>
bool access( std::string nodeName, T value, IMG hCamera)
{
  read(nodeName, value, hCamera);
  cout << "Set " << nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {
      if (typeid(value) == typeid(double)) {
        void* temp =  (void*)&value;
        double tempstr = *(double*)temp;
        result = NSetAsFloat(hNode, tempstr);
      } else if (typeid(value) == typeid(cvbbool_t)) {
        void* temp =  (void*)&value;
        cvbbool_t tempstr = *(cvbbool_t*)temp;
        result = NSetAsBoolean(hNode, tempstr);

      } else if (typeid(value) == typeid(cvbint64_t)) {
        void* temp =  (void*)&value;
        cvbint64_t tempstr = *(cvbint64_t*)temp;
        result = NSetAsInteger(hNode, tempstr);
      } else if (typeid(value) == typeid(string)) { //String is not working because compiler think its double ? run time issue
        void* temp =  (void*)&value;
        string tempstr = *(string*)temp;
        result = NSetAsString(hNode, tempstr.c_str());
      } else
        result = -1;

      if (result >= 0)
      {
        cout << "Node value set to " << value << endl;
      }
      else
      {
        cout << "Node value error: " << CVC_ERROR_FROM_HRES(result) << endl;
        return false;
      }

      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
      return false;
    }
    ReleaseObject(hNodeMap);
    return true;
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
    return false;
  }
}

bool access_int( std::string nodeName, int value, IMG hCamera)
{
  read_int(nodeName, hCamera);
  cout << "Set int " << nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {

      result = NSetAsInteger(hNode, value);


      if (result >= 0)
      {
        cout << "Node value set to " << value << endl;
      }
      else
      {
        cout << "Node value error: " << CVC_ERROR_FROM_HRES(result) << endl;
        return false;
      }

      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
      return false;
    }
    ReleaseObject(hNodeMap);
    return true;
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
    return false;
  }
}

bool access_double( std::string nodeName, double value, IMG hCamera)
{
  read_double(nodeName, hCamera);
  cout << "Set double " << nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {

      result = NSetAsFloat(hNode, value);


      if (result >= 0)
      {
        cout << "Node value set to " << value << endl;
      }
      else
      {
        cout << "Node value error: " << CVC_ERROR_FROM_HRES(result) << endl;
        return false;
      }

      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
      return false;
    }
    ReleaseObject(hNodeMap);
    return true;
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
    return false;
  }
}

bool access_bool( std::string nodeName, bool value, IMG hCamera)
{
  read_bool(nodeName, hCamera);
  cout << "Set bool " << nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {

      result = NSetAsBoolean(hNode, value);


      if (result >= 0)
      {
        cout << "Node value set to " << value << endl;
      }
      else
      {
        cout << "Node value error: " << CVC_ERROR_FROM_HRES(result) << endl;
        return false;
      }

      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
      return false;
    }
    ReleaseObject(hNodeMap);
    return true;
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
    return false;
  }
}

bool access_str( std::string nodeName, string value, IMG hCamera)
{
  read_str(nodeName, hCamera);
  cout << "Set str " << nodeName << ": ";

  NODEMAP hNodeMap = NULL;
  cvbres_t result = NMHGetNodeMap(hCamera, hNodeMap);
  if (result >= 0)
  {
    // get width feature node
    NODE hNode = NULL;
    result = NMGetNode(hNodeMap, nodeName.c_str(), hNode);
    if (result >= 0)
    {

      result = NSetAsString(hNode, value.c_str());


      if (result >= 0)
      {
        cout << "Node value set to " << value << endl;
      }
      else
      {
        cout << "Node value error: " << CVC_ERROR_FROM_HRES(result) << endl;
        return false;
      }

      ReleaseObject(hNode);
    }
    else
    {
      cout << "Node error: " << CVC_ERROR_FROM_HRES(result) << endl;
      return true;
    }
    ReleaseObject(hNodeMap);
  }
  else
  {
    cout << "Nodemap error: " << CVC_ERROR_FROM_HRES(result) << endl;
    return false;
  }
  return true;
}

bool set_camera_parameter(const std::vector<std::pair<std::string, std::string>> &cameraParameter, IMG hCamera) {
  arghandler argHandler;
  if (CanNodeMapHandle(hCamera)) {
    for (auto it = cameraParameter.begin(); it != cameraParameter.end(); ++it) {
      if (argHandler.isBoolean(it->second)) {
        if (!access_bool(it->first, argHandler.toBoolean(it->second), hCamera) ) {
            std::cerr << "Cannot set '" << it->first << "' with boolean value " << argHandler.toBoolean(it->second) << std::endl;
            return false;
        }
      }
      if (argHandler.isInteger(it->second)) {
        if (!access_int(it->first, argHandler.toInteger(it->second), hCamera) ) {
            std::cerr << "Cannot set '" << it->first << "' with integer value " << argHandler.toInteger(it->second) << std::endl;
            return false;
        }
      }
      if (argHandler.isDouble(it->second)) {
        if (!access_double(it->first, argHandler.toDouble(it->second), hCamera) ) {
            std::cerr << "Cannot set '" << it->first << "' with double value " << argHandler.toDouble(it->second) << std::endl;
            return false;
        }
      }
      if (argHandler.isString(it->second)) {
        if (!access_str(it->first, it->second, hCamera) ) {
            std::cerr << "Cannot set '" << it->first << "' with string value " << it->second << std::endl;
            return false;
        }
      }
    }
    return true;
  }
  return false;
}

}
