#include "ros/ros.h"
#include "crazyflie_driver/Orientation.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/UpdateParams.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"

//#include <regex>
#include <thread>
#include <mutex>

#include "Crazyflie.h"

constexpr double pi() { return std::atan(1)*4; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    float roll_trim,
    float pitch_trim,
    bool enable_logging)
    : m_cf(link_uri)
    , m_tf_prefix(tf_prefix)
    , m_isEmergency(false)
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_enableLogging(enable_logging)
    , m_serviceEmergency()
    , m_serviceUpdateParams()
    , m_subscribeCmdVel()
    , m_pubImu()
    , m_pubTemp()
    , m_pubMag()
    , m_pubPressure()
    , m_pubBattery()
    , m_sentSetpoint(false)
  {
    ros::NodeHandle n;
    m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
    m_serviceEmergency = n.advertiseService(tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
    m_serviceUpdateParams = n.advertiseService(tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);

    m_pubImu = n.advertise<crazyflie_driver::Orientation>(tf_prefix + "/orientation", 10);
    m_pubTemp = n.advertise<sensor_msgs::Temperature>(tf_prefix + "/temperature", 10);
    m_pubPressure = n.advertise<std_msgs::Float32>(tf_prefix + "/pressure", 10);
    m_pubBattery = n.advertise<std_msgs::Float32>(tf_prefix + "/battery", 10);

    std::thread t(&CrazyflieROS::run, this);
    t.detach();

    // initialize vbat to 3.8 (no scaling)
    rx_vbat = 3.8;
    // initialize thrust to 0
    rx_thrust = 0;
  }

private:
  struct logImu {
    float stabilizer_roll;
    float stabilizer_pitch;
    float stabilizer_yaw;
    uint16_t stabilizer_thrust;
  } __attribute__((packed));

  struct logMet {
    float baro_temp;
    float baro_pressure;
    float pm_vbat;
  } __attribute__((packed));

private:
  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;
    return true;
  }

  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.setParam<T>(id, (T)value);
  }

  float scaleThrustByBattery(float vbat){
    float vnom = 4.0;

    if( rx_thrust < 35000 ){
      vnom = 3.8;
    }else{
      vnom = 3.5;
    }
    float discharge = vbat/vnom;
    return (1/discharge);
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("Update parameters");
    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    return true;
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      float roll = msg->linear.y + m_roll_trim;
      float pitch = - (msg->linear.x + m_pitch_trim);
      float yawrate = msg->angular.z;
      uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

      // normalize thrust based on battery level
      float batScalar = scaleThrustByBattery(rx_vbat);
      thrust *= batScalar;

      ROS_INFO("new CF set point: t:%d (%.2f), p:%.2f, r:%.2f, yr:%.3f\n", thrust, batScalar, pitch, roll, yawrate);
      m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
      m_sentSetpoint = true;
    }
  }

  void run()
  {
    // m_cf.reboot();

    ROS_INFO("Requesting parameters...");
    m_cf.requestParamToc();
    for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
      auto entry = *iter;
      std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
      switch (entry.type) {
        case Crazyflie::ParamTypeUint8:
          ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
          break;
        case Crazyflie::ParamTypeInt8:
          ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
          break;
        case Crazyflie::ParamTypeUint16:
          ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
          break;
        case Crazyflie::ParamTypeInt16:
          ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
          break;
        case Crazyflie::ParamTypeUint32:
          ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
          break;
        case Crazyflie::ParamTypeInt32:
          ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
          break;
        case Crazyflie::ParamTypeFloat:
          ros::param::set(paramName, m_cf.getParam<float>(entry.id));
          break;
      }
    }

    std::unique_ptr<LogBlock<logImu> > logBlockImu;
    std::unique_ptr<LogBlock<logMet> > logBlockMet;
    if (m_enableLogging) {
      ROS_INFO("Requesting Logging variables...");
      m_cf.requestLogToc();

      std::function<void(logImu*)> cb_imu = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1);

      logBlockImu.reset(new LogBlock<logImu>(
        &m_cf,{
          {"stabilizer", "roll"},
          {"stabilizer", "pitch"},
          {"stabilizer", "yaw"},
          {"stabilizer", "thrust"},
        }, cb_imu));
      logBlockImu->start(10); // 100ms

      std::function<void(logMet*)> cb_met = std::bind(&CrazyflieROS::onMetData, this, std::placeholders::_1);

      logBlockMet.reset(new LogBlock<logMet>(
        &m_cf,{
          {"baro", "temp"},
          {"baro", "pressure"},
          {"pm",   "vbat"},
        }, cb_met));
      logBlockMet->start(100); // 1s

    }

    ROS_INFO("Ready...");

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint) {
        m_cf.sendPing();
      }
      m_sentSetpoint = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

  }

  void onImuData(logImu* data) {
    // Measured in degrees
    {
      crazyflie_driver::Orientation msg;   
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = m_tf_prefix + "/base_link";
      msg.roll = data->stabilizer_roll;
      msg.pitch = data->stabilizer_pitch;
      msg.yaw = data->stabilizer_yaw;
      msg.thrust = data->stabilizer_thrust;
      rx_thrust = msg.thrust;
      m_pubImu.publish(msg);
    }
  }

  void onMetData(logMet* data) {

    // Measured in celcius
    {
      sensor_msgs::Temperature msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = m_tf_prefix + "/base_link";
      msg.temperature = data->baro_temp;  // Celcius
      m_pubTemp.publish(msg);
    }

    // hPa (=mbar)
    {
      std_msgs::Float32 msg;
      msg.data = data->baro_pressure;
      m_pubPressure.publish(msg);
    }

    // Volts
    {
      std_msgs::Float32 msg;
      msg.data = data->pm_vbat;
      rx_vbat = data->pm_vbat;
      m_pubBattery.publish(msg);
    }
  }

private:
  Crazyflie m_cf;
  std::string m_tf_prefix;
  bool m_isEmergency;
  float m_roll_trim;
  float m_pitch_trim;
  bool m_enableLogging;

  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceUpdateParams;
  ros::Subscriber m_subscribeCmdVel;
  ros::Publisher m_pubImu;
  ros::Publisher m_pubTemp;
  ros::Publisher m_pubMag;
  ros::Publisher m_pubPressure;
  ros::Publisher m_pubBattery;

  bool m_sentSetpoint;

  float rx_vbat;
  float rx_thrust;
};

bool add_crazyflie(
  crazyflie_driver::AddCrazyflie::Request  &req,
  crazyflie_driver::AddCrazyflie::Response &res)
{
  ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d",
    req.uri.c_str(),
    req.tf_prefix.c_str(),
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging);

  // Leak intentionally
  CrazyflieROS* cf = new CrazyflieROS(
    req.uri,
    req.tf_prefix,
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_crazyflie", add_crazyflie);
  ros::spin();

  return 0;
}
