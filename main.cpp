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
    // Set default PCD file path and coloring flag
    std::string pcd_filename = "data/lidar_kitti_sample.pcd"; // Default file
    bool apply_coloring = true; // Apply color mapping to point cloud (configurable)

    // Override PCD filename if provided via command-line arguments
    if (argc > 1) {
        pcd_filename = argv[1]; 
    } else {
        std::cout << "No PCD file specified. Using default: " << pcd_filename << "\n";
    }

    // Initialize viewer with predefined configuration parameters
    PointCloudViewer viewer(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT, Config::WINDOW_TITLE);

    // Launch async thread to load and stream PCD data to the viewer
    std::thread loader_thread(loadPCDAsyncToViewer, pcd_filename, std::ref(viewer), apply_coloring);

    // Execute the main viewer loop (blocks until viewer window is closed)
    viewer.run();

    // Ensure the loader thread is properly joined before exiting
    if (loader_thread.joinable()) {
        loader_thread.join();
    }

    std::cout << "Viewer has been closed. Exiting application.\n";
    return 0;
}
