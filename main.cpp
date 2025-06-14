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
 *  - PointCloudViewer: A viewer single header that handles rendering the point cloud.
 *  - loadEventsAsyncToViewer: A function to load CSV event data asynchronously and stream it to the viewer.
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
#include <fstream>
#include <sstream>
#include <functional> // For std::ref and std::cref
#include <tbb/tbb.h>


// Function to load events from a CSV file and stream them to the viewer
// The CSV is expected to contain: x,y,polarity,timestamp
inline void loadEventsAsyncToViewer(const std::string& filename,
                                   PointCloudViewer& viewer) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open CSV file: " << filename << '\n';
        return;
    }

    std::string line;
    // Skip header if present
    if (!std::getline(file, line))
        return;

    constexpr size_t batch_size = 500; // Number of events to add per update
    std::vector<Point> batch;
    batch.reserve(batch_size);

    int prev_t = 0;

    while (std::getline(file, line) && viewer.isRunning()) {
        std::istringstream ss(line);
        int x = 0, y = 0, p = 0, t = 0;
        char comma;
        if (!(ss >> x >> comma >> y >> comma >> p >> comma >> t)) {
            continue; // Skip malformed line
        }

        // Sleep according to timestamp difference to simulate real-time
        if (prev_t != 0 && t > prev_t) {
            std::this_thread::sleep_for(std::chrono::milliseconds(t - prev_t));
        }
        prev_t = t;

        Point pt;
        pt.x = static_cast<float>(x);
        pt.y = static_cast<float>(y);
        pt.z = static_cast<float>(t);

        if (p == 0) {          // polarity 0 -> blue
            pt.r = 0; pt.g = 0; pt.b = 255;
        } else {               // polarity 1 -> red
            pt.r = 255; pt.g = 0; pt.b = 0;
        }

        batch.emplace_back(pt);
        if (batch.size() >= batch_size) {
            viewer.addPoints(batch);
            batch.clear();
        }
    }

    if (!batch.empty()) {
        viewer.addPoints(batch);
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

    // Launch async thread to load and stream CSV events to the viewer
    std::thread loader_thread(loadEventsAsyncToViewer, csv_filename, std::ref(viewer));

    // Execute the main viewer loop (blocks until viewer window is closed)
    viewer.run();

    // Ensure the loader thread is properly joined before exiting
    if (loader_thread.joinable()) {
        loader_thread.join();
    }

    std::cout << "Viewer has been closed. Exiting application.\n";
    return 0;
}
