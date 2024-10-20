/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 20.10.2024
 */

/**
 * @class PointCloudViewer
 * @brief A class for visualizing 3D point clouds using OpenGL and GLFW.
 *
 * The PointCloudViewer provides an interactive interface for rendering and 
 * navigating 3D point cloud data. It supports asynchronous data streaming, 
 * camera control, and rendering of grid and axis overlays.
 *
 * @section Initialization
 * The constructor initializes the viewer with a specified window size and title,
 * setting up OpenGL contexts and necessary buffers for rendering.
 *
 * @section Point Cloud Data
 * The viewer supports adding points asynchronously via the addPoints() method.
 * Points are represented as instances of the Point structure, which holds 3D 
 * coordinates (x, y, z) and RGB color values. Point data can be read from PCD 
 * files in binary format through the readPCD() function.
 *
 * @section Utility Structures & Functions
 * The following utility structures and functions are defined for ease of use:
 *
 * - **Point**: A structure that represents a 3D point with position (x, y, z) 
 *   and RGB color values (r, g, b). It defaults to white if no color is specified.
 *
 * - **Constants**: A structure that holds constant strings for reading PCD files.
 *   It includes prefixes for binary data and supported field names.
 *
 * - **getGLErrorString**: A utility function that translates OpenGL error codes 
 *   into readable strings for debugging purposes.
 *
 * - **HSVtoRGB**: Converts HSV color values to RGB, allowing for color mapping 
 *   based on point distance.
 *
 * - **colorPointsBasedOnDistance**: A function that colors points based on their 
 *   distance from the origin using improved gradient mapping.
 *
 * - **readPCD**: A function that reads PCD files in binary format and populates 
 *   a vector of Point structures with the data.
 *
 * @section Rendering
 * The render() function is called continuously in the main loop to update the 
 * display, including the point cloud, grid, and axes. The viewer uses OpenGL 
 * shaders for rendering, allowing for customization of visual effects.
 *
 * @section User Interaction
 * The viewer captures mouse and keyboard input for navigation and interaction:
 * - Mouse movements control camera azimuth and elevation for orbiting around the point cloud.
 * - Keyboard inputs enable panning, zooming, and toggling cursor capture mode.
 * - The R key resets the camera to its default position.
 *
 * @section Configuration
 * Configuration settings are defined in the Config namespace, allowing for easy 
 * adjustments of window size, camera settings, grid size, and point size. These 
 * parameters can be modified for different visualization needs.
 *
 * @section Thread Safety
 * The viewer is designed to be thread-safe when adding points and processing data. 
 * Mutexes and condition variables are used to manage concurrent access to shared 
 * resources.
 *
 * @section Cleanup
 * The destructor cleans up OpenGL resources, including buffers and shaders, and 
 * terminates GLFW.
 *
 * Example usage:
 * @code
 * PointCloudViewer viewer;
 * viewer.run();
 * @endcode
 *
 * @note This class requires OpenGL and GLFW libraries to be linked during 
 * compilation.
 */


#ifndef POINT_CLOUD_VIEWER_HPP
#define POINT_CLOUD_VIEWER_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <string>
#include <iterator>
#include <algorithm> // clamp
#include <array>
#include <execution> // For parallel algorithms

// Include OpenGL headers
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#else
#include <GL/glew.h>
#endif

// Include GLFW for window management
#include <GLFW/glfw3.h>

// Define PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ==========================
// Configuration Constants
// ==========================
namespace Config {
    // Window settings
    constexpr int WINDOW_WIDTH = 1920;
    constexpr int WINDOW_HEIGHT =   1080;
    constexpr char WINDOW_TITLE[] = "CloudPeek Point Cloud Viewer";

    // Grid settings
    constexpr float GRID_SIZE = 0.5f;    // Half-size for a 1m square
    constexpr float GRID_STEP = 0.05f;   // Grid cell size

    // Camera settings
    constexpr float INITIAL_DISTANCE = 10.0f;
    constexpr float INITIAL_AZIMUTH = 0.0f;
    constexpr float INITIAL_ELEVATION = 20.0f;
    constexpr float INITIAL_FOV = 45.0f;
    constexpr float CAMERA_SENSITIVITY = 0.1f;
    constexpr float ZOOM_SPEED = 0.35f;
    constexpr float PAN_SPEED = 5.0f;
    constexpr float MIN_DISTANCE = 0.5f;
    constexpr float MAX_DISTANCE = 150.0f;

    // Point rendering settings
    constexpr float POINT_SIZE = 5.0f;

    // Supported Data Fields
    constexpr std::array<const char*, 5> SUPPORTED_FIELDS = { "x", "y", "z", "rgb", "rgba" };
}

// ==========================
// Utility Structures & Functions
// ==========================

// Structure to represent a point with position and RGB color
struct Point {
    float x, y, z;
    uint8_t r = 255, g = 255, b = 255; // Default to white
};

// Structure to hold constant strings for PCD reading
struct Constants {
    inline static const std::string DATA_BINARY_PREFIX = "binary";
    inline static const std::string SUPPORTED_FIELD_X = "x";
    inline static const std::string SUPPORTED_FIELD_Y = "y";
    inline static const std::string SUPPORTED_FIELD_Z = "z";
    inline static const std::string SUPPORTED_FIELD_RGB = "rgb";
    inline static const std::string SUPPORTED_FIELD_RGBA = "rgba";
};

// Utility function to translate OpenGL error codes to strings
inline std::string getGLErrorString(GLenum error) {
    switch (error) {
        case GL_NO_ERROR: return "GL_NO_ERROR";
        case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
        case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
        case GL_STACK_OVERFLOW: return "GL_STACK_OVERFLOW";
        case GL_STACK_UNDERFLOW: return "GL_STACK_UNDERFLOW";
        case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
        default: return "Unknown OpenGL Error";
    }
}


// Function to convert a value to an HSV color and then convert to RGB
inline void HSVtoRGB(float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (s == 0.0f) {
        // Achromatic (grey)
        uint8_t gray = static_cast<uint8_t>(v * 255);
        r = g = b = gray;
        return;
    }

    h = std::fmod(h, 1.0f) * 6.0f; // sector 0 to 5
    int i = static_cast<int>(h);
    float f = h - static_cast<float>(i);
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));

    float r_f, g_f, b_f;
    switch (i) {
        case 0: r_f = v; g_f = t; b_f = p; break;
        case 1: r_f = q; g_f = v; b_f = p; break;
        case 2: r_f = p; g_f = v; b_f = t; break;
        case 3: r_f = p; g_f = q; b_f = v; break;
        case 4: r_f = t; g_f = p; b_f = v; break;
        default: r_f = v; g_f = p; b_f = q; break; // case 5
    }

    r = static_cast<uint8_t>(std::round(r_f * 255));
    g = static_cast<uint8_t>(std::round(g_f * 255));
    b = static_cast<uint8_t>(std::round(b_f * 255));
}

// Function to color points based on distance with improved gradient mapping
inline  void colorPointsBasedOnDistance(std::vector<Point>& points, float max_distance) {
    // Using parallel execution to color points efficiently
    std::for_each(std::execution::par, points.begin(), points.end(),
        [&](Point& point) {
            // Calculate the Euclidean distance from the origin
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

            // Normalize the distance with respect to the maximum distance
            float t_norm = std::min(distance / max_distance, 1.0f);  // Clamp between 0 and 1

            // Map the normalized distance to a hue value (0 to 0.66 for red to blue)
            float hue = (1.0f - t_norm) * 0.66f; // 0.0 = red, 0.66 = blue
            float saturation = 1.0f; // Full saturation
            float value = 1.0f; // Full brightness

            // Convert HSV to RGB
            HSVtoRGB(hue, saturation, value, point.r, point.g, point.b);
        }
    );
}

// Function to read PCD file (supports "DATA binary" format only)
inline bool readPCD(const std::string& filename, std::vector<Point>& points) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Could not open file; Make sure file is correct and the pcd is binary format!" << filename << '\n';
        return false;
    }

    std::string line;
    bool dataSectionFound = false;
    size_t pointCount = 0;
    size_t width = 0, height = 0;
    std::vector<std::string> fields;
    std::unordered_map<std::string, size_t> fieldIndices;

    // Parse header
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string key;
        iss >> key;

        if (key == "FIELDS") {
            fields.assign(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>());
            for (size_t i = 0; i < fields.size(); ++i) {
                fieldIndices[fields[i]] = i;
            }
        }
        else if (key == "POINTS") {
            iss >> pointCount;
        }
        else if (key == "WIDTH") {
            iss >> width;
        }
        else if (key == "HEIGHT") {
            iss >> height;
        }
        else if (key == "DATA") {
            if (line.find(Constants::DATA_BINARY_PREFIX) != std::string::npos) {
                dataSectionFound = true;
                break;
            } else {
                std::cerr << "Error: Only 'binary' DATA format is supported.\n";
                return false;
            }
        }
    }

    if (!dataSectionFound) {
        std::cerr << "Error: 'DATA binary' not found in the PCD file header.\n";
        return false;
    }

    if (pointCount == 0) {
        if (width * height == 0) {
            std::cerr << "Error: Number of points not specified in the PCD header.\n";
            return false;
        }
        pointCount = width * height;
    }

    if (fields.size() < 3) {
        std::cerr << "Error: PCD file must contain at least x, y, z fields.\n";
        return false;
    }

    // Determine indices for required fields
    size_t x_idx, y_idx, z_idx, rgb_idx = SIZE_MAX;
    try {
        x_idx = fieldIndices.at(Constants::SUPPORTED_FIELD_X);
        y_idx = fieldIndices.at(Constants::SUPPORTED_FIELD_Y);
        z_idx = fieldIndices.at(Constants::SUPPORTED_FIELD_Z);
    }
    catch (const std::out_of_range&) {
        std::cerr << "Error: PCD file must contain x, y, z fields.\n";
        return false;
    }

    // Check for RGB fields
    bool hasRGB = false;
    if (fieldIndices.find(Constants::SUPPORTED_FIELD_RGB) != fieldIndices.end()) {
        hasRGB = true;
        rgb_idx = fieldIndices.at(Constants::SUPPORTED_FIELD_RGB);
    }
    else if (fieldIndices.find(Constants::SUPPORTED_FIELD_RGBA) != fieldIndices.end()) {
        hasRGB = true;
        rgb_idx = fieldIndices.at(Constants::SUPPORTED_FIELD_RGBA);
    }

    // Calculate point size
    size_t pointSize = fields.size() * sizeof(float);

    // Read binary data
    std::vector<char> binaryData(pointCount * pointSize);
    file.read(binaryData.data(), binaryData.size());
    if (static_cast<size_t>(file.gcount()) != binaryData.size()) {
        std::cerr << "Error: Unexpected end of file while reading point data.\n";
        return false;
    }

    // Parse binary data into points
    points.reserve(pointCount);
    const char* ptr = binaryData.data();
    for (size_t i = 0; i < pointCount; ++i) {
        Point p;
        uint32_t rgbInt = 0;
        for (size_t f = 0; f < fields.size(); ++f) {
            float value = *reinterpret_cast<const float*>(ptr);
            ptr += sizeof(float);
            if (f == x_idx) p.x = value;
            else if (f == y_idx) p.y = value;
            else if (f == z_idx) p.z = value;
            else if (hasRGB && f == rgb_idx) rgbInt = *reinterpret_cast<const uint32_t*>(&value);
        }

        if (hasRGB) {
            p.r = (rgbInt >> 16) & 0xFF;
            p.g = (rgbInt >> 8) & 0xFF;
            p.b = rgbInt & 0xFF;
            // Assign default color if black
            if (p.r == 0 && p.g == 0 && p.b == 0) {
                p.r = p.g = p.b = 255;
            }
        }
        points.emplace_back(p);
    }

    std::cout << "Successfully read " << points.size() << " points from " << filename << '\n';
    return true;
}



// Simple 4x4 Matrix structure for transformations
struct Matrix4x4 {
    std::array<float, 16> data = {
        1.0f, 0.0f, 0.0f, 0.0f, 
        0.0f, 1.0f, 0.0f, 0.0f, 
        0.0f, 0.0f, 1.0f, 0.0f, 
        0.0f, 0.0f, 0.0f, 1.0f
    };

    static Matrix4x4 identity() {
    Matrix4x4 mat;
        mat.data = {
            1.0f, 0.0f, 0.0f, 0.0f, 
            0.0f, 1.0f, 0.0f, 0.0f, 
            0.0f, 0.0f, 1.0f, 0.0f, 
            0.0f, 0.0f, 0.0f, 1.0f
        };
        return mat;
    }
    
    // Matrix multiplication (Column-Major)
    Matrix4x4 operator*(const Matrix4x4& other) const {
        Matrix4x4 result;
        for(int row=0; row<4; ++row)
            for(int col=0; col<4; ++col) {
                result.data[col*4 + row] = 0.0f;
                for(int k=0; k<4; ++k)
                    result.data[col*4 + row] += data[k*4 + row] * other.data[col*4 + k];
            }
        return result;
    }
};

// Forward declaration of PointCloudViewer for callbacks
class PointCloudViewer;

// ==========================
// PointCloudViewer Class
// ==========================
class PointCloudViewer {
public:
    // Constructor with parameters
    PointCloudViewer(int width = Config::WINDOW_WIDTH, int height = Config::WINDOW_HEIGHT, const char* title = Config::WINDOW_TITLE) :
        width_(width), height_(height), title_(title), window_(nullptr),
        vbo_(0), vao_(0), color_vbo_(0), shader_program_(0),
        grid_vbo_(0), grid_vao_(0), axes_vbo_(0), axes_vao_(0),
        target_{0.0f, 0.0f, 0.0f},
        distance_(Config::INITIAL_DISTANCE),
        azimuth_(Config::INITIAL_AZIMUTH), elevation_(Config::INITIAL_ELEVATION),
        pan_x_(0.0f), pan_y_(0.0f),
        last_x_(static_cast<float>(width_) / 2.0f), last_y_(static_cast<float>(height_) / 2.0f),
        first_mouse_(true),
        fov_(Config::INITIAL_FOV),
        point_size_(Config::POINT_SIZE),
        is_running_(false),
        data_updated_(false)
    {
        init();
    }

    // Destructor
    ~PointCloudViewer() {
        cleanup();
    }

    // Add points asynchronously
    void addPoints(const std::vector<Point>& new_points) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            point_queue_.emplace(new_points);
        }
        queue_cond_var_.notify_one();
    }

    // Main loop
    void run() {
        is_running_ = true;
        // Start the data processing thread
        std::thread data_thread(&PointCloudViewer::processData, this);

        // Start the rendering loop
        while (!glfwWindowShouldClose(window_) && is_running_) {
            processInput(window_);
            render();
            glfwPollEvents();
        }

        // Stop the data processing thread
        is_running_ = false;
        queue_cond_var_.notify_one();
        if (data_thread.joinable())
            data_thread.join();
    }

    // Stop the viewer
    void stop() {
        is_running_ = false;
        queue_cond_var_.notify_one();
    }

    // Check if the viewer is running
    bool isRunning() const {
        return is_running_.load();
    }

    // Setters for width and height
    void setWidth(int width) {
        width_ = width;
    }

    void setHeight(int height) {
        height_ = height;
    }

    // Method to clear all points
    void clearPoints() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        point_cloud_.clear();
        point_colors_.clear();
        data_updated_ = true;
        
        // Optionally, update the OpenGL buffers immediately
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

private:
    // Window parameters
    int width_, height_;
    const char* title_;
    GLFWwindow* window_;

    // OpenGL objects
    GLuint vbo_, vao_, color_vbo_;
    GLuint shader_program_;

    // Grid and Axes
    GLuint grid_vbo_, grid_vao_;
    GLuint axes_vbo_, axes_vao_;

    // Shader programs for grid and axes
    GLuint grid_shader_program_;
    GLuint axes_shader_program_;

    // Point cloud data
    std::vector<float> point_cloud_;    // x, y, z
    std::vector<float> point_colors_;   // r, g, b
    std::mutex data_mutex_;
    std::atomic<bool> data_updated_;

    // Asynchronous data streaming
    std::queue<std::vector<Point>> point_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cond_var_;
    std::atomic<bool> is_running_;

    // Camera parameters for Arcball Camera
    float target_[3];
    float distance_;
    float azimuth_;
    float elevation_;
    float pan_x_, pan_y_;

    // Mouse parameters
    float last_x_, last_y_;
    bool first_mouse_;

    // Field of View
    float fov_;

    // Point rendering parameters
    float point_size_;

    // Rotation Parameters for the Grid
    float grid_rotation_x_ = 0.0f;
    float grid_rotation_y_ = 0.0f;
    float grid_rotation_z_ = 0.0f;


    // Cursor control
    bool cursor_captured_ = false; // Track cursor mode
    bool toggle_pressed_ = false;  // Debounce toggle key

    // Vertex Shader Source
    const char* vertex_shader_src_ = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;
        
        uniform mat4 MVP;
        
        out vec3 ourColor;
        
        void main(){
            gl_Position = MVP * vec4(aPos, 1.0);
            ourColor = aColor;
            gl_PointSize = 2.0;
        }
    )";

    // Fragment Shader Source
    const char* fragment_shader_src_ = R"(
        #version 330 core
        in vec3 ourColor;
        out vec4 FragColor;
        
        void main(){
            FragColor = vec4(ourColor, 1.0);
        }
    )";

    // Grid Shader Sources
    const char* grid_vertex_shader_src_ = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        
        uniform mat4 MVP;
        
        void main(){
            gl_Position = MVP * vec4(aPos, 1.0);
        }
    )";

    const char* grid_fragment_shader_src_ = R"(
        #version 330 core
        out vec4 FragColor;
        
        void main(){
            FragColor = vec4(0.5, 0.5, 0.5, 1.0); // Gray grid lines
        }
    )";

    // Axes Shader Sources
    const char* axes_vertex_shader_src_ = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;
        
        uniform mat4 MVP;
        
        out vec3 ourColor;
        
        void main(){
            gl_Position = MVP * vec4(aPos, 1.0);
            ourColor = aColor;
        }
    )";

    const char* axes_fragment_shader_src_ = R"(
        #version 330 core
        in vec3 ourColor;
        out vec4 FragColor;
        
        void main(){
            FragColor = vec4(ourColor, 1.0);
        }
    )";

    // Initialize GLFW, OpenGL, Shaders, Buffers
    void init() {
        // Initialize GLFW
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW\n";
            exit(EXIT_FAILURE);
        }

        // Set OpenGL version (3.3 Core)
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // For macOS
    #endif

        // Create window
        window_ = glfwCreateWindow(width_, height_, title_, NULL, NULL);
        if (!window_) {
            std::cerr << "Failed to create GLFW window\n";
            glfwTerminate();
            exit(EXIT_FAILURE);
        }

        glfwMakeContextCurrent(window_);

    #ifndef __APPLE__
        // Initialize GLEW
        glewExperimental = GL_TRUE;
        if (glewInit() != GLEW_OK) {
            std::cerr << "Failed to initialize GLEW\n";
            exit(EXIT_FAILURE);
        }
    #endif

        // Set callbacks
        glfwSetWindowUserPointer(window_, this);
        glfwSetFramebufferSizeCallback(window_, framebuffer_size_callback);
        glfwSetCursorPosCallback(window_, mouse_callback_dispatch);
        glfwSetScrollCallback(window_, scroll_callback_dispatch);
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            


        // Setup OpenGL state
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE); // Enable program point size if set in shader

        // Compile shaders and link programs
        shader_program_ = createShaderProgram(vertex_shader_src_, fragment_shader_src_);
        grid_shader_program_ = createShaderProgram(grid_vertex_shader_src_, grid_fragment_shader_src_);
        axes_shader_program_ = createShaderProgram(axes_vertex_shader_src_, axes_fragment_shader_src_);

        if (!shader_program_ || !grid_shader_program_ || !axes_shader_program_) {
            std::cerr << "Failed to create one or more shader programs\n";
            exit(EXIT_FAILURE);
        }

        // Generate VBO and VAO for points
        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glGenBuffers(1, &color_vbo_);

        glBindVertexArray(vao_);

        // Vertex positions
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // Vertex colors
        glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        // Setup Grid
        setupGrid();

        // Setup Axes
        setupAxes();

        // Check for OpenGL errors during initialization
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            std::cerr << "OpenGL Error during initialization: " << getGLErrorString(error) << "\n";
        }

        std::cout << "Initialization complete.\n";
    }

    // Create shader program
    GLuint createShaderProgram(const char* vertex_src, const char* fragment_src) {
        auto compileShader = [](const char* src, GLenum type) -> GLuint {
            GLuint shader = glCreateShader(type);
            glShaderSource(shader, 1, &src, nullptr);
            glCompileShader(shader);
            GLint success;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success) {
                char infoLog[512];
                glGetShaderInfoLog(shader, 512, nullptr, infoLog);
                std::cerr << "Shader Compilation Failed:\n" << infoLog << "\n";
                return 0;
            }
            return shader;
        };

        GLuint vertex_shader = compileShader(vertex_src, GL_VERTEX_SHADER);
        if (!vertex_shader) return 0;

        GLuint fragment_shader = compileShader(fragment_src, GL_FRAGMENT_SHADER);
        if (!fragment_shader) {
            glDeleteShader(vertex_shader);
            return 0;
        }

        GLuint program = glCreateProgram();
        glAttachShader(program, vertex_shader);
        glAttachShader(program, fragment_shader);
        glLinkProgram(program);

        // Check linking errors
        GLint success;
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetProgramInfoLog(program, 512, nullptr, infoLog);
            std::cerr << "Program Linking Failed:\n" << infoLog << "\n";
            glDeleteShader(vertex_shader);
            glDeleteShader(fragment_shader);
            return 0;
        }

        // Shaders can be deleted after linking
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);

        std::cout << "Shader program created successfully.\n";
        return program;
    }

    // Setup Grid Geometry (1m square) aligned with PCL coordinate system (X-Y plane, Z=0)
    void setupGrid() {
        std::vector<float> grid_vertices;
        float grid_size = Config::GRID_SIZE;    // 0.5m in each direction for a 1m square
        float grid_step = Config::GRID_STEP;   // Finer grid lines

        for (float i = -grid_size; i <= grid_size; i += grid_step) {
            // Lines parallel to Y-axis (X fixed, Z=0)
            grid_vertices.insert(grid_vertices.end(), {i, -grid_size, 0.0f, i, grid_size, 0.0f});
            // Lines parallel to X-axis (Y fixed, Z=0)
            grid_vertices.insert(grid_vertices.end(), {-grid_size, i, 0.0f, grid_size, i, 0.0f});
        }

        glGenVertexArrays(1, &grid_vao_);
        glGenBuffers(1, &grid_vbo_);

        glBindVertexArray(grid_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, grid_vbo_);
        glBufferData(GL_ARRAY_BUFFER, grid_vertices.size() * sizeof(float), grid_vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);

        std::cout << "Grid setup complete and aligned with PCL coordinate system.\n";
    }


    // Setup Axes Geometry
    void setupAxes() {
        // Define axes lines (X - Red, Y - Green, Z - Blue)
        std::vector<float> axes_vertices = {
            // X-axis (Red)
            0.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,

            // Y-axis (Green)
            0.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 0.0f,  0.0f, 1.0f, 0.0f,

            // Z-axis (Blue)
            0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1.0f,
            0.0f, 0.0f, 1.0f,  0.0f, 0.0f, 1.0f,
        };

        glGenVertexArrays(1, &axes_vao_);
        glGenBuffers(1, &axes_vbo_);

        glBindVertexArray(axes_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, axes_vbo_);
        glBufferData(GL_ARRAY_BUFFER, axes_vertices.size() * sizeof(float), axes_vertices.data(), GL_STATIC_DRAW);
        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // Color attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);

        std::cout << "Axes setup complete.\n";
    }

    // Render function
    void render() {
        // Update buffer if data has been updated
        if (data_updated_) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_);
            glBufferData(GL_ARRAY_BUFFER, point_cloud_.size() * sizeof(float), point_cloud_.data(), GL_DYNAMIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, color_vbo_);
            glBufferData(GL_ARRAY_BUFFER, point_colors_.size() * sizeof(float), point_colors_.data(), GL_DYNAMIC_DRAW);
            data_updated_ = false;
        }

        // Clear buffers
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f); // Dark background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Compute common view and projection matrices
        Matrix4x4 projection = perspective(fov_, static_cast<float>(width_) / height_, 0.5f, Config::MAX_DISTANCE * 2.0f);
        Matrix4x4 view = computeViewMatrix();

        // Draw Grid with its own MVP
        Matrix4x4 grid_model = Matrix4x4::identity();
        grid_model = grid_model * rotate(grid_rotation_x_, 1.0f, 0.0f, 0.0f);
        grid_model = grid_model * rotate(grid_rotation_y_, 0.0f, 1.0f, 0.0f);
        grid_model = grid_model * rotate(grid_rotation_z_, 0.0f, 0.0f, 1.0f);
        Matrix4x4 MVP_grid = projection * view * grid_model;

        glUseProgram(grid_shader_program_);
        glUniformMatrix4fv(glGetUniformLocation(grid_shader_program_, "MVP"), 1, GL_FALSE, MVP_grid.data.data());
        glBindVertexArray(grid_vao_);
        size_t num_lines = static_cast<size_t>((Config::GRID_SIZE / Config::GRID_STEP) * 2 + 1) * 2;
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(num_lines * 2));
        glBindVertexArray(0);

        // Draw Axes
        glUseProgram(axes_shader_program_);
        glUniformMatrix4fv(glGetUniformLocation(axes_shader_program_, "MVP"), 1, GL_FALSE, (projection * view).data.data());
        glBindVertexArray(axes_vao_);
        glDrawArrays(GL_LINES, 0, 6);
        glBindVertexArray(0);

        // Draw Point Cloud
        glUseProgram(shader_program_);
        glUniformMatrix4fv(glGetUniformLocation(shader_program_, "MVP"), 1, GL_FALSE, (projection * view).data.data());
        glBindVertexArray(vao_);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(point_cloud_.size() / 3));
        glBindVertexArray(0);

        // Swap buffers
        glfwSwapBuffers(window_);

        // Check for OpenGL errors
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            std::cerr << "OpenGL Error during rendering: " << getGLErrorString(error) << "\n";
        }
    }

    // Compute the View Matrix based on Arcball Camera parameters
    Matrix4x4 computeViewMatrix() {
        // Convert spherical coordinates to Cartesian coordinates
        float rad_azimuth = azimuth_ * M_PI / 180.0f;
        float rad_elevation = elevation_ * M_PI / 180.0f;

        float cam_x = target_[0] + distance_ * cosf(rad_elevation) * cosf(rad_azimuth);
        float cam_y = target_[1] + distance_ * cosf(rad_elevation) * sinf(rad_azimuth);
        float cam_z = target_[2] + distance_ * sinf(rad_elevation); // Updated to set Z based on elevation

        float eye[3] = { cam_x + pan_x_, cam_y + pan_y_, cam_z };
        float center[3] = { target_[0] + pan_x_, target_[1] + pan_y_, target_[2] };
        float up[3] = { 0.0f, 0.0f, 1.0f }; // Set Z-up

        return lookAt(eye, center, up);
    }

    // Perspective projection matrix
    Matrix4x4 perspective(float fov, float aspect, float near, float far) {
        Matrix4x4 mat;
        float tan_half_fov = tanf(fov * 0.5f * M_PI / 180.0f);
        mat.data[0] = 1.0f / (aspect * tan_half_fov);
        mat.data[5] = 1.0f / tan_half_fov;
        mat.data[10] = -(far + near) / (far - near);
        mat.data[11] = -1.0f;
        mat.data[14] = -(2.0f * far * near) / (far - near);
        mat.data[15] = 0.0f;
        return mat;
    }

    // LookAt view matrix
    Matrix4x4 lookAt(const float eye[3], const float center_pos[3], const float up_vec[3]) {
        float f[3] = {
            center_pos[0] - eye[0],
            center_pos[1] - eye[1],
            center_pos[2] - eye[2]
        };
        normalize(f);
        float s[3];
        cross(f, up_vec, s);
        normalize(s);
        float u[3];
        cross(s, f, u);

        Matrix4x4 mat;
        mat.data[0] = s[0];
        mat.data[1] = u[0];
        mat.data[2] = -f[0];
        mat.data[3] = 0.0f;

        mat.data[4] = s[1];
        mat.data[5] = u[1];
        mat.data[6] = -f[1];
        mat.data[7] = 0.0f;

        mat.data[8] = s[2];
        mat.data[9] = u[2];
        mat.data[10] = -f[2];
        mat.data[11] = 0.0f;

        mat.data[12] = -dot(s, eye);
        mat.data[13] = -dot(u, eye);
        mat.data[14] = dot(f, eye);
        mat.data[15] = 1.0f;

        return mat;
    }

    // Helper function to create rotation matrices
    Matrix4x4 rotate(float angle_deg, float x, float y, float z) {
        Matrix4x4 mat = Matrix4x4::identity();
        float angle_rad = angle_deg * M_PI / 180.0f;
        float c = cosf(angle_rad);
        float s = sinf(angle_rad);
        float one_c = 1.0f - c;

        mat.data[0] = x * x * one_c + c;
        mat.data[1] = y * x * one_c + z * s;
        mat.data[2] = x * z * one_c - y * s;
        mat.data[3] = 0.0f;

        mat.data[4] = x * y * one_c - z * s;
        mat.data[5] = y * y * one_c + c;
        mat.data[6] = y * z * one_c + x * s;
        mat.data[7] = 0.0f;

        mat.data[8] = x * z * one_c + y * s;
        mat.data[9] = y * z * one_c - x * s;
        mat.data[10] = z * z * one_c + c;
        mat.data[11] = 0.0f;

        mat.data[12] = 0.0f;
        mat.data[13] = 0.0f;
        mat.data[14] = 0.0f;
        mat.data[15] = 1.0f;

        return mat;
    }    


    // Vector utility functions
    void normalize(float v[3]) {
        float length = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (length > 0.0f) {
            v[0] /= length;
            v[1] /= length;
            v[2] /= length;
        }
    }

    void cross(const float a[3], const float b[3], float result[3]) {
        result[0] = a[1]*b[2] - a[2]*b[1];
        result[1] = a[2]*b[0] - a[0]*b[2];
        result[2] = a[0]*b[1] - a[1]*b[0];
    }

    float dot(const float a[3], const float b[3]) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    // Process input
    void processInput(GLFWwindow* window) {
        float dt = delta_time();
        float pan_speed = Config::PAN_SPEED * dt;
        float rotation_speed = 50.0f * dt; // degrees per second

        // Camera panning
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            pan_x_ -= pan_speed;
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            pan_x_ += pan_speed;
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            pan_y_ += pan_speed;
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            pan_y_ -= pan_speed;


        // Toggle cursor capture with F1
        if (glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS) {
            if (!toggle_pressed_) {
                cursor_captured_ = !cursor_captured_;
                if (cursor_captured_) {
                    glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                    first_mouse_ = true; // Reset mouse state
                } else {
                    glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
                }
                toggle_pressed_ = true;
            }
        } else {
            toggle_pressed_ = false;
        }


        // Reset view with R key
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
            resetCamera();

        // Close window with ESC
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window_, true);

        // Grid rotation controls
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
            grid_rotation_y_ += rotation_speed;
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
            grid_rotation_y_ -= rotation_speed;
        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
            grid_rotation_x_ += rotation_speed;
        if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
            grid_rotation_x_ -= rotation_speed;
        if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
            grid_rotation_z_ += rotation_speed;
        if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS)
            grid_rotation_z_ -= rotation_speed;
    }

    // Framebuffer size callback
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
        glViewport(0, 0, width, height);
        if (PointCloudViewer* viewer = static_cast<PointCloudViewer*>(glfwGetWindowUserPointer(window))) {
            viewer->setWidth(width);
            viewer->setHeight(height);
        }
    }

    // Mouse movement callback dispatcher
    static void mouse_callback_dispatch(GLFWwindow* window, double xpos, double ypos) {
        if (PointCloudViewer* viewer = static_cast<PointCloudViewer*>(glfwGetWindowUserPointer(window))) {
            viewer->mouse_callback(xpos, ypos);
        }
    }

    // Mouse movement callback to respect cursor mode
    void mouse_callback(double xpos, double ypos) {
        if (!cursor_captured_) return; // Do not process if cursor is not captured

        if (first_mouse_) {
            last_x_ = static_cast<float>(xpos);
            last_y_ = static_cast<float>(ypos);
            first_mouse_ = false;
            return;
        }

        float xoffset = static_cast<float>(xpos) - last_x_;
        float yoffset = last_y_ - static_cast<float>(ypos); // Reversed since y-coordinates go from bottom to top
        last_x_ = static_cast<float>(xpos);
        last_y_ = static_cast<float>(ypos);

        float sensitivity = Config::CAMERA_SENSITIVITY;
        xoffset *= sensitivity;
        yoffset *= sensitivity;

        azimuth_ += xoffset;
        elevation_ += yoffset;

        // Constrain elevation
        elevation_ = std::clamp(elevation_, -89.0f, 89.0f);
    }

    // Scroll callback dispatcher
    static void scroll_callback_dispatch(GLFWwindow* window, double xoffset, double yoffset) {
        if (PointCloudViewer* viewer = static_cast<PointCloudViewer*>(glfwGetWindowUserPointer(window))) {
            viewer->scroll_callback(yoffset);
        }
    }

    // Scroll callback
    void scroll_callback(double yoffset) {
        distance_ -= static_cast<float>(yoffset) * Config::ZOOM_SPEED; // Adjust zoom speed
        distance_ = std::clamp(distance_, Config::MIN_DISTANCE, Config::MAX_DISTANCE);  // Allow closer zoom
    }

    // Reset Camera to default position
    void resetCamera() {
        target_[0] = target_[1] = target_[2] = 0.0f;
        distance_ = Config::INITIAL_DISTANCE;
        azimuth_ = Config::INITIAL_AZIMUTH;
        elevation_ = Config::INITIAL_ELEVATION;
        pan_x_ = pan_y_ = 0.0f;
    }

    // Calculate delta time
    float delta_time() {
        static double last_time = glfwGetTime();
        double current_time = glfwGetTime();
        float delta = static_cast<float>(current_time - last_time);
        last_time = current_time;
        return delta;
    }

    // Asynchronous data processing
    void processData() {
        while (is_running_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cond_var_.wait(lock, [this]() { return !point_queue_.empty() || !is_running_; });

            while (!point_queue_.empty()) {
                auto new_points = std::move(point_queue_.front());
                point_queue_.pop();
                lock.unlock();

                // Convert Points to separate position and color vectors
                std::vector<float> positions;
                std::vector<float> colors;
                positions.reserve(new_points.size() * 3);
                colors.reserve(new_points.size() * 3);
                for (const auto& p : new_points) {
                    positions.push_back(p.x);
                    positions.push_back(p.y);
                    positions.push_back(p.z);
                    colors.push_back(p.r / 255.0f);
                    colors.push_back(p.g / 255.0f);
                    colors.push_back(p.b / 255.0f);
                }

                // Update the main point cloud data
                {
                    std::lock_guard<std::mutex> data_lock(data_mutex_);
                    point_cloud_.insert(point_cloud_.end(), positions.begin(), positions.end());
                    point_colors_.insert(point_colors_.end(), colors.begin(), colors.end());
                    data_updated_ = true;
                }

                lock.lock();
            }
        }
    }



    // Cleanup resources
    void cleanup() {
        if (vbo_) glDeleteBuffers(1, &vbo_);
        if (color_vbo_) glDeleteBuffers(1, &color_vbo_);
        if (vao_) glDeleteVertexArrays(1, &vao_);
        if (shader_program_) glDeleteProgram(shader_program_);

        // Cleanup Grid
        if (grid_vbo_) glDeleteBuffers(1, &grid_vbo_);
        if (grid_vao_) glDeleteVertexArrays(1, &grid_vao_);
        if (grid_shader_program_) glDeleteProgram(grid_shader_program_);

        // Cleanup Axes
        if (axes_vbo_) glDeleteBuffers(1, &axes_vbo_);
        if (axes_vao_) glDeleteVertexArrays(1, &axes_vao_);
        if (axes_shader_program_) glDeleteProgram(axes_shader_program_);

        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
    }
    
};

#endif // POINT_CLOUD_VIEWER_HPP
