#include <local_planner/LocalPlanner.hpp>

using namespace local_planner;
using namespace Eigen;

// Constructor
LocalPlanner::LocalPlanner() {

    // Set parameters
    max_iterations = 10000;
    max_generations = 10;
    sample_size = 4;
    velocities = {0.75, 1.0, 0.75, -1.0};
    rotations = {M_PI / 16.0, 0.0, -M_PI / 16.0, 0.0};
    max_velocity = 1.0;
    max_rotation = M_PI / 16.0;
    goal_radius = 1.0;
    sample_time = 0.5;
    sample_time_increment = 0.1;
    cost_time = 1.0;
    cost_delta_v = 0.10;
    cost_delta_u = 0.05;
    cost_reverse = 0.25;

    // Create buffer
    buffer = new Node[max_iterations*sample_size + 1];
}

// Destructor
LocalPlanner::~LocalPlanner() {
    delete[] buffer;
}

// Run path finding simulation
void LocalPlanner::run(const grid_map::GridMap &map, const Odometry &odom, const Point &goal) {

    // Starting conditions
    const Point position = odom.pose.pose.position;
    const geometry_msgs::Quaternion orientation = odom.pose.pose.orientation;
    const Twist twist = odom.twist.twist;
    
    // Save grid
    this->map = &map;

    // Initialize start node
    Node start;
    
    start.i = 0;
    start.p = -1;
    start.t = 0;

    start.x = float(position.x);
    start.y = float(position.y);
    start.w = 2.0f * atan2f(Vector3f(orientation.x, orientation.y, orientation.z).norm(), orientation.w);
    
    // Angle inverted if orientation z < 0
    if (orientation.z < 0) 
        start.w = 2.0*M_PI - start.w;

    start.v = twist.linear.x;
    start.u = twist.angular.z;

    start.g = 0.0f;
    start.f = h(start, goal);

    // Insert start node into buffer
    buffer[0] = start;

    // Create comparator function for priority queue, ordered from highest to lowest f-score
    const std::function<bool(int, int)> comp = [this](int a, int b) { return buffer[a].f > buffer[b].f; };

    // Initialize priority queue
    std::priority_queue<int, std::vector<int>, const std::function<bool(int, int)>> queue =
        std::priority_queue<int, std::vector<int>, const std::function<bool(int, int)>>(comp);

    // Add start node to queue
    queue.push(0);

    // Start iteration counter
    iteration = 0;

    // Track highest node index
    high = 0;

    // Track best node
    int best = 0;

    // Begin iterations
    while (!queue.empty() && iteration < max_iterations) {

        // Get node with lowest f score
        best = queue.top();

        // Get node from buffer
        Node node = buffer[best];

        // Remove from priority queue
        queue.pop();

        // Goal check
        if (sqrtf(powf(node.x - goal.x, 2.0) + powf(node.y - goal.y, 2.0)) <= goal_radius)
            break;

        // Generation check
        if (node.t > max_generations)
            break;

        // Create new threads for sampling
        std::thread threads[sample_size];

        // Thread response integers
        int response[sample_size];

        // Run threads
        for (int n = 0; n < sample_size; n++)
            threads[n] = std::thread(&LocalPlanner::sample, this, std::ref(node), n, std::ref(response[n]));

        // Close threads
        for (int n = 0; n < sample_size; n++) {
            // Wait for thread to finish
            threads[n].join();
            int idx = response[n];
            if (idx) {
                // If valid response, recalculate f and add to queue
                buffer[idx].f = buffer[idx].g + h(buffer[idx], goal);
                queue.push(idx);
            }
        }

        // Update highest index
        high = (++iteration)*sample_size;

    }

    // Iterations ended

    // Generate best path
    path.clear();
    for (int i = best; i != -1; i = buffer[i].p)
        path.push_back(i);

    // Reverse so path is ordered start -> best
    std::reverse(path.begin(),path.end());

    // End of run function
}

// Get best path
void LocalPlanner::getPath(Path& path) {
    // Convert path to ros type
}

// Get next twist
void LocalPlanner::getTwist(Twist& twist) {
    // Convert first velocity/rotation instruction to ros type
}

// Get all nodes in buffer as a pose
void LocalPlanner::getAllPoses(PoseArray &poses) {
    // Convert all nodes in buffer to pose ros type
}

// H-score for a given node
float LocalPlanner::h(const Node &node, const Point &goal) {
    float dx, dy, dw;
    dx = goal.x - node.x;
    dy = goal.y - node.y;
    dw = atan2f(dy,dx) - node.w;
    if (dw > M_PI || dw <= -M_PI)
        dw += dw > M_PI ? -2.0f*M_PI : 2.0*M_PI;
    return (sqrtf(dx*dx + dy*dy)/max_velocity + abs(dw)/max_rotation) * cost_time;
}

// Sampling
void LocalPlanner::sample(const Node& node, const int n, int &res) {

    // Set response to 0 (invalid)
    res = 0;

    // Copy node values
    float x = node.x;
    float y = node.y;
    float w = node.w;
    float g = node.g;

    // Get sample velocity and rotation
    const float v = velocities[n];
    const float u = velocities[u];

    // Sampling time
    float t = 0;
    while (t < sample_time) {

        // New yaw
        w += u * sample_time_increment;

        // New position
        x += v * cosf(w) * sample_time_increment;
        y += v * sinf(w) * sample_time_increment;

        // Bounds check
        grid_map::Position position(x,y);
        if (!(map->atPosition("traversable", position)))
            return; // return invalid response

        // Increase time
        t += sample_time_increment;
    }

    // Yaw angle wrapping, w = (-pi, pi]
    if (w > M_PI || w <= -M_PI)
        w += w > M_PI ? -2.0*M_PI : 2.0*M_PI;

    // Add time cost
    g += cost_time * t;

    // Add cost from delta v,u
    g += cost_delta_v * abs(v - node.v);
    g += cost_delta_u * abs(u - node.u);

    // Add cost from reversing
    if (v < 0)
        g += cost_reverse * t;

    // Calculate node index (valid response)
    res = iteration*sample_size + n + 1;

    // Copy into buffer
    buffer[res] = {x, y, w, v, u, g, 0.0f, res, node.i, node.t+1};

}