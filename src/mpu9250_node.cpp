#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "std_srvs/Trigger.h"
#include "upm/mpu9250.hpp"

using namespace upm;
using namespace std;

class ImuNode
{
public:
  ros::NodeHandle node_handle;
  ros::Publisher imu_data_pub;
  ros::Publisher temp_data_pub;
  string device;
  string frame_id;
  MPU9250 sensor{1};
  bool publish_temperature;
  double rate;

  explicit ImuNode(ros::NodeHandle nh)
    : node_handle(nh)
  {
    node_handle.param("device", device, std::string("mpu-9250"));
    node_handle.param("frame_id", frame_id, std::string("imu"));
    node_handle.param("rate", rate, 100.0);

    ROS_INFO("device: %s", device.c_str());
    ROS_INFO("frame_id: %s", frame_id.c_str());
    ROS_INFO("rate: %f [Hz]", rate);
    ROS_INFO("publish_temperature: %s", (publish_temperature ? "true" : "false"));

    imu_data_pub = node_handle.advertise<sensor_msgs::Imu>("data_raw", 100);
    if (publish_temperature)
    {
      temp_data_pub = node_handle.advertise<sensor_msgs::Temperature>("temperature", 100);
    }

  }

  /**
   * @brief Open IMU device file
   */
  bool open(void)
  {
    return sensor.init();
  }

  int publish_imu_data()
  {

    sensor_msgs::Imu data;
    data.header.frame_id = frame_id;
    data.header.stamp = ros::Time::now();

    float ax, ay, az;
    sensor.getAccelerometer(&ax, &ay, &az);
    // Linear acceleration
    data.linear_acceleration.x = ax;
    data.linear_acceleration.y = ay;
    data.linear_acceleration.z = az;


    float vx, vy, vz;
    sensor.getGyroscope(&vx, &vy, &vz);
    // Angular velocity
    data.angular_velocity.x = vx;
    data.angular_velocity.y = vy;
    data.angular_velocity.z = vz;


    float ox, oy, oz;
    sensor.getMagnetometer(&ox, &oy, &oz);
    // Orientation
    data.orientation.x = ox;
    data.orientation.y = oy;
    data.orientation.z = oz;

    imu_data_pub.publish(data);
  }

  int publish_temp_data()
  {
    sensor_msgs::Temperature data;
    data.header.frame_id = frame_id;
    data.header.stamp = ros::Time::now();

    // imu Temperature
    data.temperature = sensor.getTemperature();
    data.variance = 0;

    temp_data_pub.publish(data);
  }

  bool spin()
  {
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
      sensor.update();

      publish_imu_data();

      if (publish_temperature)
      {
        publish_temp_data();
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu");
  ros::NodeHandle nh("~");
  ImuNode node(nh);

  if (!node.open())
  {
    ROS_ERROR("Cannot open device");
  }

  node.spin();

  return (0);
}
