/*
 * CloudPeeK a Point Cloud Viewer Application
 * 
 * This application loads a PCD (Point Cloud Binary Data) file asynchronously and streams it to a PointCloudViewer. 
 * Optionally, the points can be colored based on their distance from the origin.
 *
 * Main functionalities:
 *  - Asynchronous loading and processing of PCD files
 *  - Parallel computation for performance optimization
 *  - Batch-wise streaming of points to the viewer for real-time visualization
 *  - Optional color mapping based on point distance
 * 
 * Key components:
 *  - PointCloudViewer: A viewer single header that handles rendering the point cloud.
 *  - loadPCDAsyncToViewer: A function to load PCD data asynchronously and stream it to the viewer.
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
#include <functional> // For std::ref and std::cref
#include <tbb/tbb.h>
#include <fstream>


// Simple loader for Prophesee RAW event files.
// This loader assumes each event is stored as:
//   uint32_t timestamp;
//   uint16_t x;
//   uint16_t y;
// Events are streamed in small batches to the viewer where
// x -> X axis, y -> Y axis and timestamp -> Z axis.
inline void loadRAWAsyncToViewer(const std::string& filename,
                                 PointCloudViewer& viewer) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open RAW file: " << filename << '\n';
        return;
    }

    // Skip header lines that start with '%'
    std::string line;
    while (file.peek() == '%') {
        std::getline(file, line);
    }

    constexpr size_t batch_size = 10000;
    std::vector<Point> batch;
    batch.reserve(batch_size);

    while (viewer.isRunning()) {
        uint32_t t;
        uint16_t x, y;
        file.read(reinterpret_cast<char*>(&t), sizeof(t));
        file.read(reinterpret_cast<char*>(&x), sizeof(x));
        file.read(reinterpret_cast<char*>(&y), sizeof(y));
        if (!file) break;

        Point p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(t) * 1e-6f; // scale timestamp
        batch.emplace_back(p);

        if (batch.size() >= batch_size) {
            viewer.addPoints(batch);
            batch.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (!batch.empty()) {
        viewer.addPoints(batch);
    }
}


// Optimized function to load PCD asynchronously with optional coloring
inline void loadPCDAsyncToViewer(const std::string& filename, PointCloudViewer& viewer, bool apply_coloring) {
    std::vector<Point> points;
    if (!readPCD(filename, points)) {
        std::cerr << "Failed to load PCD file: " << filename << '\n';
        return;
    }

    size_t total_points = points.size();

    // Determine the maximum distance for normalization
    float max_distance = 50.0f; // Default value meter
    if (apply_coloring) {
        // Compute the actual maximum distance in the dataset using parallel reduction
        max_distance = std::transform_reduce(
            std::execution::par,
            points.begin(),
            points.end(),
            0.0f,
            [](float a, float b) { return std::max(a, b); },
            [&](const Point& p) -> float {
                return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            }
        );

        // To avoid division by zero
        if (max_distance == 0.0f) max_distance = 1.0f;
    }

    // Split the points into smaller batches for streaming
    constexpr size_t batch_size = 10000; // Adjust as needed for real-time streaming
    size_t batches = (total_points + batch_size - 1) / batch_size;

    // Process and stream batches
    for (size_t i = 0; i < batches && viewer.isRunning(); ++i) {
        size_t start_idx = i * batch_size;
        size_t end_idx = std::min(start_idx + batch_size, total_points);
        std::vector<Point> batch(points.begin() + start_idx, points.begin() + end_idx);

        if (apply_coloring) {
            // Color the current batch of points
            colorPointsBasedOnDistance(batch, max_distance);
        }

        viewer.addPoints(batch);
        std::cout << "Added batch " << i + 1 << "/" << batches 
                  << " with " << batch.size() << " points.\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Simulate streaming delay
    }
}


int main(int argc, char* argv[]) {
    // Default input file
    std::string input_file = "data/raw/hand_spinner.raw";
    bool apply_coloring = true; // Used for PCD files

    // Override input file if provided via command-line arguments
    if (argc > 1) {
        input_file = argv[1];
    } else {
        std::cout << "No input file specified. Using default: " << input_file << "\n";
    }

    // Initialize viewer with predefined configuration parameters
    PointCloudViewer viewer(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT, Config::WINDOW_TITLE);

    // Launch async thread based on file type
    std::thread loader_thread;
    if (input_file.size() >= 4 && input_file.substr(input_file.size() - 4) == ".raw") {
        loader_thread = std::thread(loadRAWAsyncToViewer, input_file, std::ref(viewer));
    } else {
        loader_thread = std::thread(loadPCDAsyncToViewer, input_file, std::ref(viewer), apply_coloring);
    }

    // Execute the main viewer loop (blocks until viewer window is closed)
    viewer.run();

    // Ensure the loader thread is properly joined before exiting
    if (loader_thread.joinable()) {
        loader_thread.join();
    }

    std::cout << "Viewer has been closed. Exiting application.\n";
    return 0;
}
