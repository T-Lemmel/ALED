#include <aquabot_ekf/boat.h>
#include <aquabot_ekf/ally.h>
#include <aquabot_ekf/gps2enu.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

namespace aquabot_ekf
{

// main node
class GPS2Pose : public rclcpp::Node
{
public:
  GPS2Pose() : Node("gps2pose"), boat{this, &enu}
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    static auto gps_sub = create_subscription<NavSatFix>("/wamv/sensors/gps/gps/fix", 5, [this](NavSatFix::UniquePtr msg)
    {boat.updatePose(*msg);});

    static auto imu_sub = create_subscription<Imu>("/wamv/sensors/imu/imu/data", 1, [this](Imu::UniquePtr msg)
    {boat.updateImu(*msg);});

    static auto buoy_sub = create_subscription<ParamVec>("/wamv/sensors/acoustics/receiver/range_bearing", 1, [this](ParamVec::UniquePtr msg)
    {boat.updateBuoy(*msg);});

    static auto ally_sub = create_subscription<PoseArray>("/wamv/ais_sensor/allies_positions", 1, [this](PoseArray::UniquePtr msg)
    {processAllies(*msg);});
  }


private:
  toENU enu;

  Boat boat;

  std::vector<Ally> allies;

  void processAllies(const PoseArray &msg)
  {
    if(allies.size() < msg.poses.size())
    {
     allies.reserve(msg.poses.size());
     for(uint _ = allies.size(); _ < msg.poses.size(); ++_)
       allies.emplace_back(this, &enu);
    }

    for(uint i = 0; i < msg.poses.size(); ++i)
      allies[i].updatePose(msg.poses[i]);
  }

};

}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<aquabot_ekf::GPS2Pose>());
    rclcpp::shutdown();
    return 0;
  }
