/*
 * CloudPeeK a Point Cloud Viewer Application
 * 
 * This application loads event data from a CSV file and streams it to a PointCloudViewer.
 * Points are colored in real-time based on their polarity value.
 *
 * Main functionalities:
 *  - Asynchronous loading of event data from a CSV file
 *  - Parallel computation for performance optimization
 *  - Batch-wise streaming of events to the viewer for real-time visualization
 *  - Coloring of points based on event polarity
 * 
 * Key components:
 *  - PointCloudViewer: A single-header viewer that handles rendering the point cloud.
 *  - loadEventsAsyncToViewer: A function to load CSV event data asynchronously and stream it to the viewer.
 *
 * New in this version:
 *  - Dragging with the middle mouse button also rotates the view when the cursor is free.
 * 
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 20.10.2024
 */

#include "PointCloudViewer.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <fstream>
#include <sstream>
#include <functional> // For std::ref and std::cref
#include <tbb/tbb.h>


// Function to load events from a CSV file and stream them to the viewer
// The CSV is expected to contain: x,y,polarity,timestamp
inline void loadEventsAsyncToViewer(const std::string& filename,
                                   PointCloudViewer& viewer,
                                   int time_window_ms) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open CSV file: " << filename << '\n';
        return;
    }

    std::string line;
    // Skip header if present
    if (!std::getline(file, line))
        return;

    struct TimedPoint { Point pt; int ts; };
    std::deque<TimedPoint> window_points;

    int prev_t = -1;

    while (std::getline(file, line) && viewer.isRunning()) {
        std::istringstream ss(line);
        int x = 0, y = 0, p = 0, t = 0;
        char comma;
        if (!(ss >> x >> comma >> y >> comma >> p >> comma >> t)) {
            continue; // Skip malformed line
        }


        // Sleep according to timestamp difference to simulate real-time
        if (prev_t >= 0 && t > prev_t) {
            std::this_thread::sleep_for(std::chrono::milliseconds(t - prev_t));
        }
        prev_t = t;

        Point pt;
        // Swap x and y axes when mapping from CSV to points
        pt.x = static_cast<float>(y / 100.0);
        pt.y = static_cast<float>(x / 100.0);
        pt.z = static_cast<float>(0.0); // Assuming z is always 0 for 2D events

        if (p == 0) {          // polarity 0 -> blue
            pt.r = 0; pt.g = 0; pt.b = 255;
        } else {               // polarity 1 -> red
            pt.r = 255; pt.g = 0; pt.b = 0;
        }

        // Add new event and remove expired ones
        window_points.push_back({pt, t});
        while (!window_points.empty() &&
               (t - window_points.front().ts > time_window_ms)) {
            window_points.pop_front();
        }

        // Convert window points to vector for the viewer
        std::vector<Point> current_points;
        current_points.reserve(window_points.size());
        for (const auto& wp : window_points) {
            current_points.push_back(wp.pt);
        }

        viewer.setPoints(current_points);
    }

}


int main(int argc, char* argv[]) {
    // Set default CSV file path
    std::string csv_filename = "data/csv/events.csv";

    // Override CSV filename if provided via command-line arguments
    if (argc > 1) {
        csv_filename = argv[1];
    } else {
        std::cout << "No CSV file specified. Using default: " << csv_filename << "\n";
    }

    // Initialize viewer with predefined configuration parameters
    PointCloudViewer viewer(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT, Config::WINDOW_TITLE);

    std::cout << "[Info] Hold the middle mouse button and drag to rotate when the cursor is free." << std::endl;
    std::cout << "[Info] Right-click and drag to pan the point cloud." << std::endl;

    int time_window_ms = 100; // default time window
    if (argc > 2) {
        time_window_ms = std::stoi(argv[2]);
    }

    // Launch async thread to load and stream CSV events to the viewer
    std::thread loader_thread(loadEventsAsyncToViewer, csv_filename, std::ref(viewer), time_window_ms);

    // Execute the main viewer loop (blocks until viewer window is closed)
    viewer.run();

    // Ensure the loader thread is properly joined before exiting
    if (loader_thread.joinable()) {
        loader_thread.join();
    }

    std::cout << "Viewer has been closed. Exiting application.\n";
    return 0;
}
