#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "ee3305_nav/ee3305_nav.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    /** Publishes the raw occupancy grid data, and the global costmap that has inflation cost values. */
    class Controller : public rclcpp::Node
    {

    private:
        // ----------- Publishers / Subscribers --------------
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;

        // ----------- Timers -------------------
        rclcpp::TimerBase::SharedPtr timer_main; // contains the timer that runs the main looping function at regular intervals.

        // ----------- Parameters ---------------
        double frequency;
        double lookahead_distance;
        double stop_thres;
        double lookahead_lin_vel;
        double max_lin_vel;
        double max_lin_acc;
        double max_ang_acc;
        double max_ang_vel;

        // ----------- States / Others -------------
        std::vector<double> path_flat;
        double prev_lin_vel;
        double rbt_x;
        double rbt_y;
        double rbt_h;
        double prev_time;
        double prev_ang_vel;
        double integral_lin, prev_error_lin, integral_ang, prev_error_ang;

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit Controller()
            : Node("controller")
        {
            initStates();
            initParams();
            initTopics();
            initTimers();
        }

    private:
        void initStates()
        {
            rbt_x = NAN;
            rbt_y = NAN;
            rbt_h = NAN;
            prev_lin_vel = 0;
            prev_time = this->now().seconds();
            prev_ang_vel = 0;
            integral_lin = 0.0;
            prev_error_lin = 0.0;
            integral_ang = 0.0;
            prev_error_ang = 0.0;
        }

        /** Initializes and read parameters, if any. */
        void initParams()
        {
            initParam(this, "frequency", frequency);
            initParam(this, "lookahead_distance", lookahead_distance);
            initParam(this, "stop_thres", stop_thres);
            initParam(this, "lookahead_lin_vel", lookahead_lin_vel);
            initParam(this, "max_lin_vel", max_lin_vel);
            initParam(this, "max_lin_acc", max_lin_acc);
            initParam(this, "max_ang_acc", max_ang_acc);
            initParam(this, "max_ang_vel", max_ang_vel);
        }

        /** Initializes topics and messages, if any. */
        void initTopics()
        {
            sub_path = this->create_subscription<nav_msgs::msg::Path>(
                "path", rclcpp::SensorDataQoS(),
                std::bind(&Controller::cbPath, this, std::placeholders::_1));

            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", rclcpp::SensorDataQoS(),
                std::bind(&Controller::cbOdom, this, std::placeholders::_1));

            pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel", rclcpp::SystemDefaultsQoS());
        }

        void cbPath(nav_msgs::msg::Path::SharedPtr msg)
        {
            path_flat.clear();
            for (const geometry_msgs::msg::PoseStamped &pose : msg->poses)
            {
                path_flat.push_back(pose.pose.position.x);
                path_flat.push_back(pose.pose.position.y);
            }
        }

        void cbOdom(nav_msgs::msg::Odometry::SharedPtr msg)
        { // transform assumes zero transform between `map` frame and `odom` frame.
            rbt_x = msg->pose.pose.position.x;
            rbt_y = msg->pose.pose.position.y;
            const auto &q = msg->pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y + q.y + q.z * q.z);
            rbt_h = atan2(siny_cosp, cosy_cosp);
        }

        /** Initializes the timers with their callbacks.*/
        void initTimers()
        {
            timer_main = this->create_wall_timer(
                1s / frequency,
                std::bind(&Controller::cbTimerMain, this));
        }

        void publishStopMessage()
        {
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            pub_cmd_vel->publish(stop_msg);
        }

        /** The function that is run at regular intervals */
        void cbTimerMain()
        {

            if (path_flat.empty() || std::isnan(rbt_x))
            {
                publishStopMessage();
                return;
            }
            double goal_x = path_flat[path_flat.size() - 2]; // last point on the path
            double goal_y = path_flat[path_flat.size() - 1]; // second last y point
            double distance_to_goal = std::sqrt(std::pow(goal_x - rbt_x, 2) + std::pow(goal_y - rbt_y, 2));
            if (distance_to_goal < stop_thres)
            {
                publishStopMessage();
                RCLCPP_INFO(this->get_logger(), "Target reached. Stopping the robot.");
                return;
            }
            // Find the closest point on the path to the robot
            int ci = 0;
            double closest = 1E99;
            for (int i = 0; i < static_cast<long>(path_flat.size()); i += 2)
            {
                double dx = path_flat[i] - rbt_x;
                double dy = path_flat[i + 1] - rbt_y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist < closest)
                {
                    closest = dist;
                    ci = i;
                }
            }
            double look_x = NAN;
            double look_y = NAN;
            for (int i = ci; i < static_cast<long>(path_flat.size()); i += 2)
            {
                double dx = path_flat[i] - rbt_x;
                double dy = path_flat[i + 1] - rbt_y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist > lookahead_distance)
                {
                    look_x = path_flat[i];
                    look_y = path_flat[i + 1];
                    // RCLCPP_WARN(this->get_logger(), "dx %f, dy %f", dx, dy);

                    break;
                }
            }
            if (std::isnan(look_x) || std::isnan(look_y))
            {

                return;
            }

            double dx = look_x - rbt_x;
            double dy = look_y - rbt_y;

            // Calculate the angle to the lookahead point
            double angle_to_lookahead = atan2(dy, dx); // dy and dx are differences to lookahead point
            double heading_error = std::abs(angle_to_lookahead) - std::abs(rbt_h);

            // Normalize heading error to [-pi, pi]
            heading_error = atan2(sin(heading_error), cos(heading_error));

            // Check if the point is behind the robot or more than M_PI/4 away
            bool move_backward = std::abs(heading_error) > M_PI / 4;

            double local_x = std::cos(rbt_h) * dx + std::sin(rbt_h) * dy;
            double local_y = -std::sin(rbt_h) * dx + std::cos(rbt_h) * dy;

            // Calculate curvature
            double curvature = 2 * local_y / (local_x * local_x + local_y * local_y);

            // Getting Elasped time
            double current_time = this->now().seconds();
            double elapsed_time = current_time - prev_time;

            // Check for valid elapsed_time
            if (elapsed_time <= 0)
            {

                RCLCPP_WARN(this->get_logger(), "Elapsed time is non-positive. Check time update logic.");
                return;
            }
            prev_time = current_time;

            // Calculate constrained linear acceleration
            double lin_acc = (lookahead_lin_vel - prev_lin_vel) / elapsed_time;

            lin_acc = std::clamp(lin_acc, -max_lin_acc, max_lin_acc);

            // Calculate constrained linear velocity
            double lin_vel = prev_lin_vel + lin_acc * elapsed_time;
            lin_vel = std::clamp(lin_vel, -max_lin_vel, max_lin_vel);

            // update previous
            prev_lin_vel = lin_vel;

            // Calculate angular acceleration
            double ang_vel = curvature * lin_vel;
            double ang_acc = (ang_vel - prev_ang_vel) / elapsed_time;
            ang_acc = std::clamp(ang_acc, -max_ang_acc, max_ang_acc);

            // Calculate constrained angular velocity
            ang_vel = prev_ang_vel + ang_acc * elapsed_time;
            ang_vel = std::clamp(ang_vel, -max_ang_vel, max_ang_vel);
            prev_ang_vel = ang_vel;

            // Log angular velocity details
            // double heading_tolerance = M_PI / 10;

            if (move_backward)
            {
                RCLCPP_WARN(this->get_logger(), "Movebackwards is triggered");

                lin_vel = 0;
            }

            // Publish message
            geometry_msgs::msg::Twist msg;
            msg.angular.x = 0;             // redundancy
            msg.angular.y = 0;             // redundancy
            msg.angular.z = ang_vel * 0.5; // Check if scaling factor is needed
            msg.linear.x = lin_vel * 0.3;  // Use calculated `lin_vel` for consistency
            msg.linear.y = 0;              // redundancy
            msg.linear.z = 0;              // redundancy
            pub_cmd_vel->publish(msg);
        }
    };
}

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee3305::Controller>());
    rclcpp::shutdown();
    return 0;
}