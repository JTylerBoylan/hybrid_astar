#ifndef LOCAL_PLANNER_HPP
#define LOCAL_PLANNER_HPP

#include <ros/node_handle.h>
#include <grid_map_core/GridMap.hpp>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>

#include <queue>
#include <vector>
#include <thread>

using namespace nav_msgs;
using namespace geometry_msgs;

namespace local_planner {

    class LocalPlanner {

        public:

            // Constructor
            LocalPlanner();

            // Destructor
            ~LocalPlanner();

            // Run path finding simulation
            void run(const grid_map::GridMap &map, const Odometry &odom, const Point &goal);

            // Get best path
            void getPath(Path& path);

            // Get next twist
            void getTwist(Twist& twist);

            // Get all nodes in buffer as a pose
            void getAllPoses(PoseArray &poses);

        private:

            // Struct to hold node information
            struct Node {
                float x, y, w;  // Position
                float v, u;     // Velocity
                float g, f;     // Heuristic
                int i, p;       // Index
                int t;          // Generation
            };

            // H-score for a given node
            float h(const Node &node, const Point &goal);

            // Sampling
            void sample(const Node& node, const int n, int &res);

            // Pointer to buffer array
            Node * buffer;

            // Reference to occupancy grid
            const grid_map::GridMap * map;

            // Latest generated path
            std::vector<int> path;

            // Store the iteration of the run
            int iteration;

            // Store index of highest node in buffer
            int high;

            // Parameters
            int max_iterations;
            int max_generations;
            int sample_size;
            std::vector<float> velocities;
            std::vector<float> rotations;
            float max_velocity;
            float max_rotation;
            float goal_radius;
            float sample_time;
            float sample_time_increment;
            float cost_time;
            float cost_delta_v;
            float cost_delta_u;
            float cost_reverse;   

    };

}


#endif