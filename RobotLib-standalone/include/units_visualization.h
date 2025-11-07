// ============================================================================
// units_visualization.h - SDL2-Based Robot Visualization
// ============================================================================
// Purpose: Visualize robot simulation in real-time
//
// Requirements: SDL2 library
//   Ubuntu/Debian: sudo apt-get install libsdl2-dev
//   macOS: brew install sdl2
//   Windows: Download from libsdl.org
//
// Features:
// - Real-time 2D visualization
// - Robot, obstacles, sensors rendering
// - Path tracing
// - Debug overlays
//
// Usage: Link with -lSDL2 when compiling
// ============================================================================

#ifndef ROBOTICS_UNITS_VISUALIZATION_H
#define ROBOTICS_UNITS_VISUALIZATION_H

#include "units_simulation.h"
#include <SDL2/SDL.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

namespace robotlib {
namespace visualization {

using namespace simulation;

// ============================================================================
// Simple 2D Visualizer using SDL2
// ============================================================================
class RobotVisualizer {
private:
    SDL_Window* window_;
    SDL_Renderer* renderer_;
    int windowWidth_;
    int windowHeight_;
    double scale_;  // Pixels per meter
    bool running_;

    // Path tracing
    std::vector<Point2D> path_;
    bool traceEnabled_;
    size_t maxPathPoints_;

    // Colors (RGBA)
    struct Color {
        uint8_t r, g, b, a;
        Color(uint8_t r_ = 255, uint8_t g_ = 255, uint8_t b_ = 255, uint8_t a_ = 255)
            : r(r_), g(g_), b(b_), a(a_) {}
    };

    Color bgColor_;
    Color robotColor_;
    Color obstacleColor_;
    Color pathColor_;
    Color gridColor_;

public:
    RobotVisualizer(
        int width = 800,
        int height = 600,
        double scale = 100.0,  // 100 pixels per meter
        const char* title = "RobotLib Simulator")
        : window_(nullptr)
        , renderer_(nullptr)
        , windowWidth_(width)
        , windowHeight_(height)
        , scale_(scale)
        , running_(false)
        , traceEnabled_(true)
        , maxPathPoints_(1000)
        , bgColor_(240, 240, 240, 255)
        , robotColor_(50, 100, 255, 255)
        , obstacleColor_(100, 100, 100, 255)
        , pathColor_(255, 100, 100, 200)
        , gridColor_(200, 200, 200, 255)
    {
        init(title);
    }

    ~RobotVisualizer() {
        cleanup();
    }

    // ========================================
    // Initialization
    // ========================================

    bool init(const char* title) {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            return false;
        }

        window_ = SDL_CreateWindow(
            title,
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            windowWidth_,
            windowHeight_,
            SDL_WINDOW_SHOWN
        );

        if (!window_) {
            return false;
        }

        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
        if (!renderer_) {
            return false;
        }

        running_ = true;
        return true;
    }

    void cleanup() {
        if (renderer_) {
            SDL_DestroyRenderer(renderer_);
            renderer_ = nullptr;
        }
        if (window_) {
            SDL_DestroyWindow(window_);
            window_ = nullptr;
        }
        SDL_Quit();
    }

    // ========================================
    // Event Handling
    // ========================================

    bool handleEvents() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running_ = false;
                return false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running_ = false;
                    return false;
                }
                if (event.key.keysym.sym == SDLK_t) {
                    traceEnabled_ = !traceEnabled_;
                }
                if (event.key.keysym.sym == SDLK_c) {
                    path_.clear();
                }
            }
        }
        return true;
    }

    bool isRunning() const { return running_; }

    // ========================================
    // Coordinate Conversion
    // ========================================

    int worldToScreenX(double x) const {
        return (int)(x * scale_);
    }

    int worldToScreenY(double y) const {
        // SDL Y-axis is inverted (top = 0)
        return windowHeight_ - (int)(y * scale_);
    }

    double screenToWorldX(int x) const {
        return x / scale_;
    }

    double screenToWorldY(int y) const {
        return (windowHeight_ - y) / scale_;
    }

    // ========================================
    // Drawing Functions
    // ========================================

    void clear() {
        SDL_SetRenderDrawColor(renderer_, bgColor_.r, bgColor_.g, bgColor_.b, bgColor_.a);
        SDL_RenderClear(renderer_);
    }

    void drawGrid(double cellSize = 0.5) {
        SDL_SetRenderDrawColor(renderer_, gridColor_.r, gridColor_.g, gridColor_.b, gridColor_.a);

        // Vertical lines
        for (double x = 0; x <= screenToWorldX(windowWidth_); x += cellSize) {
            int sx = worldToScreenX(x);
            SDL_RenderDrawLine(renderer_, sx, 0, sx, windowHeight_);
        }

        // Horizontal lines
        for (double y = 0; y <= screenToWorldY(0); y += cellSize) {
            int sy = worldToScreenY(y);
            SDL_RenderDrawLine(renderer_, 0, sy, windowWidth_, sy);
        }
    }

    void drawRobot(const DifferentialDriveSimulator& robot) {
        double x = robot.getX();
        double y = robot.getY();
        double theta = robot.getTheta();
        double radius = robot.getRobotRadius();

        int cx = worldToScreenX(x);
        int cy = worldToScreenY(y);
        int r = (int)(radius * scale_);

        // Draw robot body (filled circle)
        SDL_SetRenderDrawColor(renderer_, robotColor_.r, robotColor_.g, robotColor_.b, robotColor_.a);
        drawFilledCircle(cx, cy, r);

        // Draw direction indicator (line from center)
        int dx = cx + (int)(std::cos(theta) * r);
        int dy = cy - (int)(std::sin(theta) * r);  // Negative because Y is inverted

        SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
        SDL_RenderDrawLine(renderer_, cx, cy, dx, dy);

        // Draw wheels
        double wheelLen = radius * 0.5;
        double wheelWidth = radius * 0.2;
        drawWheel(x, y, theta, wheelLen, wheelWidth, true);   // Left wheel
        drawWheel(x, y, theta, wheelLen, wheelWidth, false);  // Right wheel

        // Add to path
        if (traceEnabled_) {
            path_.push_back(Point2D(x, y));
            if (path_.size() > maxPathPoints_) {
                path_.erase(path_.begin());
            }
        }
    }

    void drawWheel(double robotX, double robotY, double robotTheta,
                   double wheelLen, double wheelWidth, bool isLeft) {
        double wheelbase = 0.15;  // Assuming standard wheelbase
        double side = isLeft ? 1.0 : -1.0;

        // Wheel position (perpendicular to robot direction)
        double wx = robotX + side * (wheelbase / 2) * std::cos(robotTheta + M_PI / 2);
        double wy = robotY + side * (wheelbase / 2) * std::sin(robotTheta + M_PI / 2);

        // Wheel endpoints
        double dx = std::cos(robotTheta) * wheelLen / 2;
        double dy = std::sin(robotTheta) * wheelLen / 2;

        int x1 = worldToScreenX(wx - dx);
        int y1 = worldToScreenY(wy - dy);
        int x2 = worldToScreenX(wx + dx);
        int y2 = worldToScreenY(wy + dy);

        SDL_SetRenderDrawColor(renderer_, 50, 50, 50, 255);
        // Draw thicker line for wheel
        for (int i = -2; i <= 2; i++) {
            SDL_RenderDrawLine(renderer_, x1, y1 + i, x2, y2 + i);
        }
    }

    void drawObstacle(const Rectangle& obs) {
        int x = worldToScreenX(obs.x - obs.width / 2);
        int y = worldToScreenY(obs.y + obs.height / 2);
        int w = (int)(obs.width * scale_);
        int h = (int)(obs.height * scale_);

        SDL_Rect rect = {x, y, w, h};

        SDL_SetRenderDrawColor(renderer_, obstacleColor_.r, obstacleColor_.g, obstacleColor_.b, obstacleColor_.a);
        SDL_RenderFillRect(renderer_, &rect);

        // Draw border
        SDL_SetRenderDrawColor(renderer_, 50, 50, 50, 255);
        SDL_RenderDrawRect(renderer_, &rect);
    }

    void drawPath() {
        if (path_.size() < 2) return;

        SDL_SetRenderDrawColor(renderer_, pathColor_.r, pathColor_.g, pathColor_.b, pathColor_.a);

        for (size_t i = 1; i < path_.size(); i++) {
            int x1 = worldToScreenX(path_[i - 1].x);
            int y1 = worldToScreenY(path_[i - 1].y);
            int x2 = worldToScreenX(path_[i].x);
            int y2 = worldToScreenY(path_[i].y);

            SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
        }
    }

    void drawSensorRay(const DifferentialDriveSimulator& robot, double angle = 0) {
        double x = robot.getX();
        double y = robot.getY();
        double theta = robot.getTheta() + angle;
        double distance = robot.getUltrasonicDistance(angle);

        int x1 = worldToScreenX(x);
        int y1 = worldToScreenY(y);
        int x2 = worldToScreenX(x + distance * std::cos(theta));
        int y2 = worldToScreenY(y + distance * std::sin(theta));

        SDL_SetRenderDrawColor(renderer_, 255, 200, 0, 150);
        SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);

        // Draw hit point
        drawFilledCircle(x2, y2, 3);
    }

    void drawText(const std::string& text, int x, int y) {
        // Simple text rendering would require SDL_ttf
        // For now, we'll skip text rendering (can be added later)
        // This is just a placeholder
    }

    void drawInfo(const DifferentialDriveSimulator& robot, double fps) {
        // Draw info text in corner (requires SDL_ttf for proper implementation)
        // For now, we'll just note what info would be shown:
        // - Position (x, y, theta)
        // - Velocities
        // - FPS
        // - Instructions
    }

    // ========================================
    // Complete Frame Rendering
    // ========================================

    void render(const DifferentialDriveSimulator& robot, double fps = 0) {
        clear();
        drawGrid();

        // Draw obstacles
        for (const auto& obs : robot.getObstacles()) {
            drawObstacle(obs);
        }

        // Draw path
        drawPath();

        // Draw sensor rays
        drawSensorRay(robot, 0);  // Forward sensor

        // Draw robot
        drawRobot(robot);

        // Present
        SDL_RenderPresent(renderer_);
    }

    void present() {
        SDL_RenderPresent(renderer_);
    }

    // ========================================
    // Helper: Draw filled circle
    // ========================================

    void drawFilledCircle(int cx, int cy, int radius) {
        for (int w = 0; w < radius * 2; w++) {
            for (int h = 0; h < radius * 2; h++) {
                int dx = radius - w;
                int dy = radius - h;
                if ((dx * dx + dy * dy) <= (radius * radius)) {
                    SDL_RenderDrawPoint(renderer_, cx + dx, cy + dy);
                }
            }
        }
    }

    // ========================================
    // Path Control
    // ========================================

    void enableTrace(bool enable) { traceEnabled_ = enable; }
    void clearPath() { path_.clear(); }

    // ========================================
    // Getters
    // ========================================

    int getWindowWidth() const { return windowWidth_; }
    int getWindowHeight() const { return windowHeight_; }
    double getScale() const { return scale_; }
};

} // namespace visualization
} // namespace robotlib

#endif // ROBOTICS_UNITS_VISUALIZATION_H
