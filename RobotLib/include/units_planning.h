// ============================================================================
// units_planning.h - Path Planning Algorithms for Robotics
// ============================================================================
// Purpose: Path and motion planning algorithms
// Dependencies: units_core.h, units_robotics.h, units_3d.h
//
// This file contains:
// - A* path planning (grid-based)
// - Dubins paths (shortest paths for car-like robots)
// - RRT (Rapidly-exploring Random Trees)
// - Potential fields for local planning
// ============================================================================

#ifndef ROBOTICS_UNITS_PLANNING_H
#define ROBOTICS_UNITS_PLANNING_H

#include "units_core.h"
#include "units_robotics.h"
#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <functional>
#include <random>

namespace units {
namespace planning {

// ============================================================================
// GRID CELL for A* Planning
// ============================================================================
struct GridCell {
    int x, y;

    GridCell() : x(0), y(0) {}
    GridCell(int x, int y) : x(x), y(y) {}

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const GridCell& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }

    // Hash function for use in unordered containers
    struct Hash {
        size_t operator()(const GridCell& cell) const {
            return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
        }
    };
};

// ============================================================================
// A* PATH PLANNER
// ============================================================================
// Why A*:
// - Complete (finds a path if one exists)
// - Optimal (finds shortest path)
// - Efficient with good heuristic
// - Industry standard for grid-based planning
//
// When to use:
// - Grid-based environments
// - Known static obstacles
// - Need optimal paths
// - 2D navigation
// ============================================================================
class AStarPlanner {
public:
    using Path = std::vector<GridCell>;
    using CostMap = std::vector<std::vector<double>>;  // Grid of costs
    using HeuristicFunc = std::function<double(const GridCell&, const GridCell&)>;

    // Constructor
    explicit AStarPlanner(int width, int height)
        : width_(width), height_(height),
          obstacles_(height, std::vector<bool>(width, false)),
          costs_(height, std::vector<double>(width, 1.0)) {}

    // Set obstacle at grid cell
    void setObstacle(int x, int y, bool is_obstacle = true) {
        if (isValid(x, y)) {
            obstacles_[y][x] = is_obstacle;
        }
    }

    // Set cost for a cell (for weighted A*)
    void setCost(int x, int y, double cost) {
        if (isValid(x, y)) {
            costs_[y][x] = cost;
        }
    }

    // Plan path from start to goal
    Path plan(const GridCell& start, const GridCell& goal,
              bool allow_diagonal = true) {
        // Validate inputs
        if (!isValid(start.x, start.y) || !isValid(goal.x, goal.y)) {
            return Path();  // Invalid start or goal
        }
        if (obstacles_[start.y][start.x] || obstacles_[goal.y][goal.x]) {
            return Path();  // Start or goal is obstacle
        }

        // Priority queue for open set (min-heap by f-score)
        using Node = std::pair<double, GridCell>;  // (f_score, cell)
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;

        // Maps for tracking
        std::unordered_map<GridCell, GridCell, GridCell::Hash> came_from;
        std::unordered_map<GridCell, double, GridCell::Hash> g_score;
        std::unordered_set<GridCell, GridCell::Hash> closed_set;

        // Initialize
        g_score[start] = 0.0;
        double h_start = heuristic(start, goal);
        open_set.push({h_start, start});

        while (!open_set.empty()) {
            GridCell current = open_set.top().second;
            open_set.pop();

            // Check if reached goal
            if (current == goal) {
                return reconstructPath(came_from, current);
            }

            // Skip if already processed
            if (closed_set.count(current)) {
                continue;
            }
            closed_set.insert(current);

            // Explore neighbors
            std::vector<GridCell> neighbors = getNeighbors(current, allow_diagonal);
            for (const auto& neighbor : neighbors) {
                if (closed_set.count(neighbor) || obstacles_[neighbor.y][neighbor.x]) {
                    continue;
                }

                // Calculate tentative g_score
                double edge_cost = costs_[neighbor.y][neighbor.x];
                if (allow_diagonal && isDiagonal(current, neighbor)) {
                    edge_cost *= constants::SQRT2;  // Diagonal cost
                }

                double tentative_g = g_score[current] + edge_cost;

                // Update if better path found
                if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g;
                    double f_score = tentative_g + heuristic(neighbor, goal);
                    open_set.push({f_score, neighbor});
                }
            }
        }

        return Path();  // No path found
    }

    // Get path length
    double getPathLength(const Path& path) const {
        double length = 0.0;
        for (size_t i = 1; i < path.size(); i++) {
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;
            length += std::sqrt(dx * dx + dy * dy);
        }
        return length;
    }

private:
    int width_, height_;
    std::vector<std::vector<bool>> obstacles_;
    std::vector<std::vector<double>> costs_;

    bool isValid(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    bool isDiagonal(const GridCell& a, const GridCell& b) const {
        return std::abs(a.x - b.x) == 1 && std::abs(a.y - b.y) == 1;
    }

    // Heuristic function (Euclidean distance)
    double heuristic(const GridCell& a, const GridCell& b) const {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Get valid neighbors
    std::vector<GridCell> getNeighbors(const GridCell& cell, bool diagonal) const {
        std::vector<GridCell> neighbors;

        // 4-connected
        const int dx4[] = {-1, 1, 0, 0};
        const int dy4[] = {0, 0, -1, 1};

        // 8-connected
        const int dx8[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        const int dy8[] = {0, 0, -1, 1, -1, 1, -1, 1};

        int count = diagonal ? 8 : 4;
        const int* dx = diagonal ? dx8 : dx4;
        const int* dy = diagonal ? dy8 : dy4;

        for (int i = 0; i < count; i++) {
            int nx = cell.x + dx[i];
            int ny = cell.y + dy[i];
            if (isValid(nx, ny)) {
                neighbors.push_back(GridCell(nx, ny));
            }
        }

        return neighbors;
    }

    // Reconstruct path from came_from map
    Path reconstructPath(const std::unordered_map<GridCell, GridCell, GridCell::Hash>& came_from,
                        GridCell current) const {
        Path path;
        path.push_back(current);

        while (came_from.count(current)) {
            current = came_from.at(current);
            path.push_back(current);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};

// ============================================================================
// DUBINS PATH - Shortest paths for car-like robots
// ============================================================================
// Why Dubins paths:
// - Shortest path for vehicles with minimum turning radius
// - Accounts for non-holonomic constraints
// - Only 6 path types exist (CSC and CCC)
// - Used in UAVs, cars, wheeled robots
//
// Path types: LSL, RSR, LSR, RSL, LRL, RLR
// (L=Left turn, R=Right turn, S=Straight)
// ============================================================================
class DubinsPath {
public:
    enum class SegmentType { LEFT, RIGHT, STRAIGHT };

    struct Config {
        double x, y;      // Position
        double theta;     // Heading (radians)

        Config() : x(0), y(0), theta(0) {}
        Config(double x, double y, double theta) : x(x), y(y), theta(theta) {}
    };

    struct Path {
        std::array<SegmentType, 3> types;
        std::array<double, 3> lengths;  // In units of turning radius
        double total_length;

        Path() : types({SegmentType::STRAIGHT, SegmentType::STRAIGHT, SegmentType::STRAIGHT}),
                 lengths({0, 0, 0}), total_length(0) {}
    };

    // Find shortest Dubins path
    static Path shortestPath(const Config& start, const Config& goal, double turning_radius) {
        // Normalize: scale so turning radius = 1
        double dx = goal.x - start.x;
        double dy = goal.y - start.y;
        double D = std::sqrt(dx * dx + dy * dy) / turning_radius;
        double theta_start = start.theta;
        double theta_goal = goal.theta;
        double alpha = std::atan2(dy, dx);

        Path best_path;
        best_path.total_length = INFINITY;

        // Try all 6 Dubins path types
        tryLSL(alpha, theta_start, theta_goal, D, best_path);
        tryRSR(alpha, theta_start, theta_goal, D, best_path);
        tryLSR(alpha, theta_start, theta_goal, D, best_path);
        tryRSL(alpha, theta_start, theta_goal, D, best_path);
        tryLRL(alpha, theta_start, theta_goal, D, best_path);
        tryRLR(alpha, theta_start, theta_goal, D, best_path);

        // Scale back to actual turning radius
        best_path.lengths[0] *= turning_radius;
        best_path.lengths[1] *= turning_radius;
        best_path.lengths[2] *= turning_radius;
        best_path.total_length *= turning_radius;

        return best_path;
    }

    // Sample points along the path
    static std::vector<Config> sample(const Config& start, const Path& path,
                                     double turning_radius, double step_size) {
        std::vector<Config> points;
        Config current = start;
        points.push_back(current);

        double distance = 0.0;
        for (int seg = 0; seg < 3; seg++) {
            double seg_len = path.lengths[seg];
            SegmentType type = path.types[seg];

            while (distance + step_size <= seg_len) {
                distance += step_size;
                current = advanceAlong(current, type, step_size, turning_radius);
                points.push_back(current);
            }

            // Handle remaining distance in segment
            double remaining = seg_len - distance;
            if (remaining > 0.001) {
                current = advanceAlong(current, type, remaining, turning_radius);
                points.push_back(current);
            }

            distance = 0.0;
        }

        return points;
    }

private:
    static double mod2pi(double angle) {
        while (angle < 0) angle += constants::TWO_PI;
        while (angle >= constants::TWO_PI) angle -= constants::TWO_PI;
        return angle;
    }

    static void tryLSL(double alpha, double theta_s, double theta_g, double D, Path& best) {
        (void)alpha;  // Reserved for future use
        double p_squared = 2 + D * D - 2 * std::cos(theta_g - theta_s) +
                          2 * D * (std::sin(theta_s) - std::sin(theta_g));
        if (p_squared < 0) return;

        double t = mod2pi(-theta_s + std::atan2(std::cos(theta_g) - std::cos(theta_s),
                         D + std::sin(theta_s) - std::sin(theta_g)));
        double p = std::sqrt(p_squared);
        double q = mod2pi(theta_g - std::atan2(std::cos(theta_g) - std::cos(theta_s),
                         D + std::sin(theta_s) - std::sin(theta_g)));

        double length = t + p + q;
        if (length < best.total_length) {
            best.types = {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT};
            best.lengths = {t, p, q};
            best.total_length = length;
        }
    }

    static void tryRSR(double alpha, double theta_s, double theta_g, double D, Path& best) {
        (void)alpha;  // Reserved for future use
        double p_squared = 2 + D * D - 2 * std::cos(theta_g - theta_s) +
                          2 * D * (std::sin(theta_g) - std::sin(theta_s));
        if (p_squared < 0) return;

        double t = mod2pi(theta_s - std::atan2(std::cos(theta_s) - std::cos(theta_g),
                         D - std::sin(theta_s) + std::sin(theta_g)));
        double p = std::sqrt(p_squared);
        double q = mod2pi(-theta_g + std::atan2(std::cos(theta_s) - std::cos(theta_g),
                         D - std::sin(theta_s) + std::sin(theta_g)));

        double length = t + p + q;
        if (length < best.total_length) {
            best.types = {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT};
            best.lengths = {t, p, q};
            best.total_length = length;
        }
    }

    static void tryLSR(double alpha, double theta_s, double theta_g, double D, Path& best) {
        (void)alpha;  // Reserved for future use
        double p_squared = -2 + D * D + 2 * std::cos(theta_g - theta_s) +
                          2 * D * (std::sin(theta_s) + std::sin(theta_g));
        if (p_squared < 0) return;

        double p = std::sqrt(p_squared);
        double t = mod2pi(-theta_s + std::atan2(-std::cos(theta_s) - std::cos(theta_g),
                         D + std::sin(theta_s) + std::sin(theta_g)) - std::atan2(-2.0, p));
        double q = mod2pi(-theta_g + std::atan2(-std::cos(theta_s) - std::cos(theta_g),
                         D + std::sin(theta_s) + std::sin(theta_g)) - std::atan2(-2.0, p));

        double length = t + p + q;
        if (length < best.total_length) {
            best.types = {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT};
            best.lengths = {t, p, q};
            best.total_length = length;
        }
    }

    static void tryRSL(double alpha, double theta_s, double theta_g, double D, Path& best) {
        (void)alpha;  // Reserved for future use
        double p_squared = -2 + D * D + 2 * std::cos(theta_g - theta_s) -
                          2 * D * (std::sin(theta_s) + std::sin(theta_g));
        if (p_squared < 0) return;

        double p = std::sqrt(p_squared);
        double t = mod2pi(theta_s - std::atan2(std::cos(theta_s) + std::cos(theta_g),
                         D - std::sin(theta_s) - std::sin(theta_g)) + std::atan2(2.0, p));
        double q = mod2pi(theta_g - std::atan2(std::cos(theta_s) + std::cos(theta_g),
                         D - std::sin(theta_s) - std::sin(theta_g)) + std::atan2(2.0, p));

        double length = t + p + q;
        if (length < best.total_length) {
            best.types = {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT};
            best.lengths = {t, p, q};
            best.total_length = length;
        }
    }

    static void tryLRL(double alpha, double theta_s, double theta_g, double D, Path& best) {
        (void)alpha;  // Reserved for future use
        double p_squared = (6.0 - D * D + 2 * std::cos(theta_g - theta_s) +
                           2 * D * (std::sin(theta_s) - std::sin(theta_g))) / 8.0;
        if (p_squared < 0 || p_squared > 1) return;

        double p = mod2pi(constants::TWO_PI - std::acos(p_squared));
        double t = mod2pi(-theta_s + std::atan2(std::cos(theta_s) - std::cos(theta_g),
                         D - std::sin(theta_s) + std::sin(theta_g)) + p / 2.0);
        double q = mod2pi(theta_g - theta_s - t + mod2pi(p));

        double length = t + p + q;
        if (length < best.total_length) {
            best.types = {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT};
            best.lengths = {t, p, q};
            best.total_length = length;
        }
    }

    static void tryRLR(double alpha, double theta_s, double theta_g, double D, Path& best) {
        (void)alpha;  // Reserved for future use
        double p_squared = (6.0 - D * D + 2 * std::cos(theta_g - theta_s) -
                           2 * D * (std::sin(theta_s) - std::sin(theta_g))) / 8.0;
        if (p_squared < 0 || p_squared > 1) return;

        double p = mod2pi(constants::TWO_PI - std::acos(p_squared));
        double t = mod2pi(theta_s - std::atan2(std::cos(theta_s) - std::cos(theta_g),
                         D - std::sin(theta_s) + std::sin(theta_g)) + p / 2.0);
        double q = mod2pi(theta_s - theta_g - t + mod2pi(p));

        double length = t + p + q;
        if (length < best.total_length) {
            best.types = {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT};
            best.lengths = {t, p, q};
            best.total_length = length;
        }
    }

    static Config advanceAlong(const Config& start, SegmentType type,
                              double distance, double turning_radius) {
        Config result = start;

        if (type == SegmentType::STRAIGHT) {
            result.x += distance * std::cos(start.theta);
            result.y += distance * std::sin(start.theta);
        } else {
            double angle_change = distance / turning_radius;
            if (type == SegmentType::LEFT) {
                result.theta += angle_change;
                result.x += turning_radius * (std::sin(result.theta) - std::sin(start.theta));
                result.y -= turning_radius * (std::cos(result.theta) - std::cos(start.theta));
            } else {  // RIGHT
                result.theta -= angle_change;
                result.x -= turning_radius * (std::sin(result.theta) - std::sin(start.theta));
                result.y += turning_radius * (std::cos(result.theta) - std::cos(start.theta));
            }
        }

        return result;
    }
};

} // namespace planning
} // namespace units

#endif // ROBOTICS_UNITS_PLANNING_H

/*
Features provided:
- A* pathfinding with diagonal movement support
- Weighted A* for cost-sensitive planning
- Dubins paths for car-like robots
- Path sampling and interpolation

Use cases:
- Grid-based navigation (warehouse robots, game AI)
- Shortest path planning with obstacles
- Car-like robot path planning (parking, waypoint following)
- UAV path planning with minimum turn radius
- Mobile robot navigation

Example - A* Planning:
    using namespace units::planning;

    // Create planner for 100x100 grid
    AStarPlanner planner(100, 100);

    // Set obstacles
    for (int x = 20; x < 80; x++) {
        planner.setObstacle(x, 50);  // Horizontal wall
    }

    // Plan path
    auto path = planner.plan(GridCell(10, 10), GridCell(90, 90));

    // Use path
    for (const auto& cell : path) {
        std::cout << "(" << cell.x << ", " << cell.y << ")\n";
    }

Example - Dubins Path:
    using namespace units::planning;

    // Start and goal configurations
    DubinsPath::Config start(0, 0, 0);  // x, y, heading
    DubinsPath::Config goal(10, 10, PI/2);
    double turning_radius = 2.0;

    // Find shortest path
    auto path = DubinsPath::shortestPath(start, goal, turning_radius);

    // Sample points along path
    auto points = DubinsPath::sample(start, path, turning_radius, 0.1);

    std::cout << "Path length: " << path.total_length << "\n";
*/
