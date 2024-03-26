#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TestPathPlannerNode : public rclcpp::Node
{
public:
    TestPathPlannerNode() : Node("test_path_planner_node")
    {
        // Publishers for map, start position, and goal position
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mapping", 10);
        goal_pub = this->create_publisher<geometry_msgs::msg::Pose>("target_position", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Timer to periodically publish test data
        timer_ = this->create_wall_timer(4s, std::bind(&TestPathPlannerNode::timer_callback, this));


    }

private:
    void timer_callback()
    {
        auto map = generate_test_map();
        auto start = generate_test_position();
        auto goal = generate_test_target_pos();

        // Publish the test data
        publish_transforms();
        //map_pub->publish(map);
        //start_pub->publish(start);
        goal_pub->publish(goal);
    }

    void publish_transforms() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "world";
        t.child_frame_id = "map";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t);
    }

    nav_msgs::msg::OccupancyGrid generate_test_map()
    {
        // Generate a test map
        nav_msgs::msg::OccupancyGrid map;

        //Pour voir la map dans Rviz
        set_parameter(rclcpp::Parameter("use_sim_time",true));
        map.header.frame_id = "map";

        std::vector<int8_t> map_data = generateRandomMap();

        // Assign the test data to the map
        map.data = map_data;

        // Set the map metadata
        map.info.resolution = 1.0; // Example resolution
        map.info.width = 100;        // Width of the map (number of columns)
        map.info.height = 100;       // Height of the map (number of rows)
        map.info.origin.position.x = 0.0;  // x position of the bottom left corner of the map
        map.info.origin.position.y = 0.0;  // y position of the bottom left corner of the map
        map.info.origin.position.z = 0.0;  // z position of the bottom left corner of the map
        map.info.origin.orientation.w = 1.0; // Orientation of the map

        // Return the populated map
        return map;
    }

    std::vector<int8_t> generateRandomMap() {
        const int width = 100;
        const int height = 100;
        const int size = width * height;
        const int num_obstacles = 20; // Number of obstacles
        const int obstacle_max_size = 15; // Maximum size of an obstacle

        std::vector<int8_t> map_data(size, 0);

        // Initialize random seed:
        std::srand(std::time(nullptr));

        for (int obs = 0; obs < num_obstacles; ++obs) {
            // Randomly select the center point of the obstacle
            int center_x = std::rand() % width;
            int center_y = std::rand() % height;
            // Randomly determine the size of the obstacle
            int obstacle_width = std::rand() % obstacle_max_size + 1;
            int obstacle_height = std::rand() % obstacle_max_size + 1;

            // Create the obstacle around the center point
            for (int i = -obstacle_height / 2; i <= obstacle_height / 2; ++i) {
                for (int j = -obstacle_width / 2; j <= obstacle_width / 2; ++j) {
                    int x = center_x + j;
                    int y = center_y + i;
                    // Check if the calculated position is within the bounds of the map
                    if (x >= 0 && x < width && y >= 0 && y < height) {
                        map_data[y * width + x] = 1;
                    }
                }
            }
        }

        return map_data;
    }

    geometry_msgs::msg::Pose generate_test_position()
    {
        // Generate a test pose
        geometry_msgs::msg::Pose pose;
        // Fill in the pose details
        pose.position.set__x(0);
        pose.position.set__y(0);
        return pose;
    }

    geometry_msgs::msg::Pose generate_test_target_pos()
    {
        // Generate a test pose
        geometry_msgs::msg::Pose pose;
        // Fill in the pose details
        pose.position.set__x(95);
        pose.position.set__y(95);
        return pose;
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
