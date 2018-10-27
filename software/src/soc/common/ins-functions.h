/*
ins-functions.hxx
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef INS_FUNCTIONS_HXX_
#define INS_FUNCTIONS_HXX_

#include "uNavINS.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree2.h"
#include "generic-function.h"
#include <sys/time.h>
#include <iostream>

/*
15 State EKF INS - 15 State Extended Kalman Filter Inertial Navigation System,
Estimates inertial position and velocity and estimates gyro and accelerometer bias
using GPS and IMU input
Example JSON configuration:
{
  "Type": "EKF15StateINS",
  "Output": "OutputName",
  "Time": X,
  "GPS": X,
  "IMU": X
}
Where:
   * Output gives a convenient name for the block (i.e. EKF).
   * Time is the time data source
   * GPS is the GPS data source
   * IMU is the IMU data source
*/
class Ekf15StateIns: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear();
  private:
    struct Config {
      ElementPtr t;
      ElementPtr GpsTow;
      ElementPtr GpsFix;
      ElementPtr GpsVn, GpsVe, GpsVd;
      ElementPtr GpsLat, GpsLon, GpsAlt;
      ElementPtr ImuGx, ImuGy, ImuGz;
      ElementPtr ImuAx, ImuAy, ImuAz;
      ElementPtr ImuHx, ImuHy, ImuHz;
    };
    struct Data {
      ElementPtr Mode;
      ElementPtr Ax, Ay, Az;
      ElementPtr Gx, Gy, Gz;
      ElementPtr Axb, Ayb, Azb;
      ElementPtr Gxb, Gyb, Gzb;
      ElementPtr Pitch, Roll, Yaw, Heading, Track;
      ElementPtr Lat, Lon, Alt;
      ElementPtr Vn, Ve, Vd;
    };
    Config config_;
    Data data_;
    bool Initialized_ = false;
    std::string ModeKey_;
    std::string AxKey_,AyKey_,AzKey_;
    std::string GxKey_,GyKey_,GzKey_;
    std::string AxbKey_,AybKey_,AzbKey_;
    std::string GxbKey_,GybKey_,GzbKey_;
    std::string PitchKey_,RollKey_,YawKey_,HeadingKey_,TrackKey_;
    std::string LatKey_,LonKey_,AltKey_;
    std::string VnKey_,VeKey_,VdKey_;
    uNavINS uNavINS_;
};

#endif
