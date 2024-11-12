#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ee3305_nav/ee3305_nav.hpp"
#include "ee3305_nav/planner.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    // =================================================================================================
    /** Handles the global path planning */
    class Planner : public rclcpp::Node
    {

    private:
        // ----------- Publishers / Subscribers --------------
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr wall_cost;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;

        // ----------- Services -------------------
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_get_plan;

        // ----------- Parameters ---------------
        double xy_tolerance;
        int doorway_tolerance;

        // ----------- States / Others -------------
        std::vector<int8_t> map;
        std::vector<int8_t> walls;
        double resolution;
        double origin_x;
        double origin_y;
        int rows;
        int cols;

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit Planner()
            : rclcpp::Node("planner")
        {
            initStates();
            initParams();
            initTopics();
            initServices();
        }

    private:
        void initStates()
        {
            resolution = 0;
            origin_x = 0;
            origin_y = 0;
            cols = 0;
        }

        /** Initializes and read parameters, if any. */
        void initParams()
        {
            initParam(this, "xy_tolerance", xy_tolerance);
            initParam(this, "doorway_tolerance", doorway_tolerance);
        }

        /** Initializes topics and messages, if any. */
        void initTopics()
        {
            auto qos = rclcpp::SensorDataQoS();
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            qos.keep_last(1);

            sub_global_costmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "global_costmap", qos,
                std::bind(&Planner::cbGlobalCostmap, this, std::placeholders::_1));

            wall_cost = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "map", qos,
                std::bind(&Planner::cbWallCost, this, std::placeholders::_1));

            qos = rclcpp::SensorDataQoS();
            pub_path = this->create_publisher<nav_msgs::msg::Path>(
                "path", qos);
        }

        void cbWallCost(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            walls = msg->data;
        }

        void cbGlobalCostmap(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
            map = msg->data;
            resolution = msg->info.resolution;
            origin_x = msg->info.origin.position.x;
            origin_y = msg->info.origin.position.y;
            rows = msg->info.height;
            cols = msg->info.width;
        }

        void initServices()
        {
            srv_get_plan = this->create_service<nav_msgs::srv::GetPlan>(
                "get_plan",
                std::bind(&Planner::cbSrvGetPlan, this, std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default);
        }

        /** Service server that returns a path in the map coordinates */
        void cbSrvGetPlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                          std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "GetPlan service request received.");
            while (map.empty())
            { // wait until a map is published
                RCLCPP_WARN_STREAM(this->get_logger(), "global_costmap topic is not published yet. Waiting for a global costmap to be published into the topic.");
                rclcpp::sleep_for(100ms);
                rclcpp::spin_some(this->get_node_base_interface());
            }

            double start_x = request->start.pose.position.x;
            double start_y = request->start.pose.position.y;
            double goal_x = request->goal.pose.position.x;
            double goal_y = request->goal.pose.position.y;

            int start_i = floor((start_x - origin_x) / resolution);
            int start_j = floor((start_y - origin_y) / resolution);
            int goal_i = floor((goal_x - origin_x) / resolution);
            int goal_j = floor((goal_y - origin_y) / resolution);

            double dx = goal_x - start_x;
            double dy = goal_y - start_y;
            double distance_to_goal = std::sqrt(dx * dx + dy * dy);
            std::vector<int> path_flat;
            if (distance_to_goal <= xy_tolerance)
            {
                RCLCPP_WARN(this->get_logger(), "Within xy tolerance. No path needed.");
                path_flat = {};
            }
            else
            {

                path_flat = run(start_i, start_j, goal_i, goal_j);
            }

            // convert to map coordinates and write to response.
            // the response is from start to goal.
            // `path_flat` is from goal to start.
            nav_msgs::msg::Path path;
            // std::cout << "path: ";
            for (int p = path_flat.size() - 2; p >= 0; p -= 2)
            {
                double i = path_flat[p];
                double j = path_flat[p + 1];

                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = "map";
                pose_stamped.header.stamp = this->now();
                pose_stamped.pose.position.x = (i * resolution) + origin_x + resolution / 2;
                pose_stamped.pose.position.y = (j * resolution) + origin_y + resolution / 2;
                // std::cout << pose_stamped.pose.position.x << "," << pose_stamped.pose.position.y << ";  ";

                pose_stamped.pose.position.z = 0;
                pose_stamped.pose.orientation.x = 0;
                pose_stamped.pose.orientation.y = 0;
                pose_stamped.pose.orientation.z = 0;
                pose_stamped.pose.orientation.w = 1;

                path.poses.push_back(pose_stamped);
            }
            // std::cout << std::endl;
            path.header.frame_id = "map";

            // publish to topic
            pub_path->publish(path);

            // write to response
            response->plan = path;
        }

        /** The main path finding algorithm */
        std::vector<int> run(
            int start_i, int start_j, int goal_i, int goal_j)
        {

            if (map.empty() || rows <= 0 || cols <= 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid map");
                return {};
            }

            auto getIndex = [&](int i, int j) -> int
            {
                return j * cols + i;
            };
            std::vector<PlannerNode> nodes_storage(rows * cols, PlannerNode(0, 0));
            std::vector<PlannerNode *> node_ptrs(rows * cols, nullptr);
            for (int j = 0; j < rows; j++)
            {
                for (int i = 0; i < cols; i++)
                {
                    nodes_storage[getIndex(i, j)] = PlannerNode(i, j);
                    node_ptrs[getIndex(i, j)] = &nodes_storage[getIndex(i, j)];
                }
            }

            std::vector<int> path;
            OpenList pq = OpenList();

            auto inMap = [&](int i, int j) -> bool
            {
                return i >= 0 && j >= 0 && i < cols && j < rows;
            };
            auto getNode = [&](int i, int j) -> PlannerNode *
            {
                if (!inMap(i, j))
                    return nullptr;
                return node_ptrs[getIndex(i, j)];
            };

            // implement heuristic function of Astar as lambda
            auto eucDist = [&](int x1, int y1, int x2, int y2) -> double
            {
                return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            };

            auto isNearObstacle = [&](int i, int j) -> bool
            {
                // Check surroundings for narrow passage
                int obstacle_count = 0;

                for (int dx = -doorway_tolerance; dx <= doorway_tolerance; ++dx)
                {
                    for (int dy = -doorway_tolerance; dy <= doorway_tolerance; ++dy)
                    {
                        if (dx == 0 && dy == 0)
                            continue;
                        if (inMap(i + dx, j + dy) && walls[getIndex(i + dx, j + dy)] > 50) // <50 indicates open space
                            obstacle_count++;
                    }
                }
                // Assuming doorways are narrow passages with 2+ open cells surrounded by obstacles
                return obstacle_count > 1;
            };

            auto getPenalty = [&](PlannerNode *from, PlannerNode *to) -> double
            {
                double distance_weight = 0.3; // Weight for distance traveled
                double turn_weight = 1.0;     // Weight for turns/angle changes
                double proximity_weight = 4.0;

                if (!inMap(to->i, to->j))
                    return std::numeric_limits<double>::infinity();

                int8_t wall = walls[getIndex(to->i, to->j)];
                int8_t cost = map[getIndex(to->i, to->j)];
                // No go zones (walls)
                if (wall >= 100 || cost >= 99)
                    return std::numeric_limits<double>::infinity();

                // Movement cost
                double movement_cost = std::sqrt(std::pow(to->i - from->i, 2) + std::pow(to->j - from->j, 2));

                // Turn penalty (calculated from angle difference)
                double angle_change = std::atan2(to->j - from->j, to->i - from->i) -
                                      std::atan2(from->j - (from->parent ? from->parent->j : from->j),
                                                 from->i - (from->parent ? from->parent->i : from->i));
                double turn_penalty = turn_weight * std::abs(angle_change);

                // proximity to walls
                double proximity_penalty = 0;
                if (isNearObstacle(to->i, to->j))
                {
                    proximity_penalty = 100;
                }

                return 1.0 + ((cost + movement_cost * distance_weight + turn_penalty * turn_weight + proximity_penalty * proximity_weight) / 100.0);
            };
            if (!inMap(start_i, start_j) || !inMap(goal_i, goal_j))
            {
                RCLCPP_ERROR(this->get_logger(), "Range : row : %d  cols: %d", rows, cols);
                RCLCPP_ERROR(this->get_logger(), "Start(%d,%d) or goal(%d,%d) position out of map bounds", start_i, start_j, goal_i, goal_j);

                return {};
            }

            PlannerNode *start_node = getNode(start_i, start_j);
            if (!start_node)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get start node");
                return {};
            }

            start_node->g = 0;
            start_node->f = eucDist(start_i, start_j, goal_i, goal_j);

            pq.queue(start_node);

            while (!pq.empty())
            {
                PlannerNode *curr_node = pq.poll();
                if (!curr_node)
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid node pointer");
                    continue;
                }
                double dx = curr_node->i - goal_i;
                double dy = curr_node->j - goal_j;
                double dist_to_goal = std::sqrt(dx * dx + dy * dy);
                if (curr_node->visited)
                {
                    continue;
                }
                else if (dist_to_goal <= xy_tolerance)
                {

                    PlannerNode *node;
                    for (node = curr_node; node != nullptr; node = node->parent)
                    {
                        path.push_back(node->i);
                        path.push_back(node->j);
                    }
                    return path;
                }
                else
                {
                    curr_node->visited = true;
                    // access neighbours
                    std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

                    for (auto [dx, dy] : directions)
                    {
                        int nx = curr_node->i + dx;
                        int ny = curr_node->j + dy;
                        if (!inMap(nx, ny))
                            continue;

                        PlannerNode *n = getNode(nx, ny);
                        if (n == nullptr || n->visited)
                            continue;

                        // new cost
                        double movement_cost = std::sqrt(dx * dx + dy * dy);
                        double new_g_cost = curr_node->g + getPenalty(curr_node, n) + movement_cost;
                        if (new_g_cost < n->g)
                        {
                            n->g = new_g_cost;
                            n->f = new_g_cost + eucDist(n->i, n->j, goal_i, goal_j);
                            n->parent = curr_node;

                            pq.queue(n);
                        }
                    };
                }
            }
            return path;
        }
    };
}

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ee3305::Planner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}