#include <rclcpp/rclcpp.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <iomanip>
#include <map>
#include <chrono>
#include <filesystem>
#include <Eigen/Eigen>

namespace fs = std::filesystem;

// Simple B-spline evaluator (uniform cubic B-spline)
class SimpleBsplineEvaluator {
public:
    SimpleBsplineEvaluator(const std::vector<Eigen::Vector3d>& ctrl_pts,
                          const std::vector<double>& knots, int order)
        : control_points_(ctrl_pts), knots_(knots), order_(order) {}

    Eigen::Vector3d evaluate(double t) {
        // For simplicity, use linear interpolation between control points
        // A full de Boor algorithm implementation would be more accurate
        int n = control_points_.size();
        if (n == 0) return Eigen::Vector3d::Zero();
        if (n == 1) return control_points_[0];

        // Normalize t to [0, 1]
        double duration = knots_.back() - knots_[order_];
        if (duration <= 0) return control_points_[0];

        double u = std::min(std::max(t / duration, 0.0), 1.0);
        double idx = u * (n - 1);

        int i0 = static_cast<int>(std::floor(idx));
        int i1 = std::min(i0 + 1, n - 1);
        double alpha = idx - i0;

        return (1.0 - alpha) * control_points_[i0] + alpha * control_points_[i1];
    }

    double getDuration() {
        if (knots_.size() <= static_cast<size_t>(order_)) return 0.0;
        return knots_.back() - knots_[order_];
    }

private:
    std::vector<Eigen::Vector3d> control_points_;
    std::vector<double> knots_;
    int order_;
};

// Per-drone file handler
struct DroneFileHandler {
    std::ofstream file_stream;
    int line_number = 1;

    DroneFileHandler() = default;
    DroneFileHandler(DroneFileHandler&&) = default;
    DroneFileHandler& operator=(DroneFileHandler&&) = default;

    ~DroneFileHandler() {
        if (file_stream.is_open()) {
            file_stream.close();
        }
    }
};

class MultiDroneTrajectoryRecorder : public rclcpp::Node {
public:
    MultiDroneTrajectoryRecorder() : Node("trajectory_recorder") {
        // Declare parameters
        this->declare_parameter("num_drones", 1);
        this->declare_parameter("output_folder", "trajectories");
        this->declare_parameter("sampling_dt", 0.05);
        this->declare_parameter("default_rgb", std::vector<int64_t>{255, 255, 255});
        this->declare_parameter("record_height", 3.0);

        num_drones_ = this->get_parameter("num_drones").as_int();
        std::string output_folder_base = this->get_parameter("output_folder").as_string();
        sampling_dt_ = this->get_parameter("sampling_dt").as_double();
        auto rgb = this->get_parameter("default_rgb").as_integer_array();
        default_rgb_ = {static_cast<int>(rgb[0]), static_cast<int>(rgb[1]), static_cast<int>(rgb[2])};
        record_height_ = this->get_parameter("record_height").as_double();

        // Create output folder with datetime
        output_folder_ = createOutputFolder(output_folder_base);

        if (output_folder_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create output folder!");
            return;
        }

        global_start_time_ = this->now();

        // Subscribe to all drones
        for (int drone_id = 1; drone_id <= num_drones_; ++drone_id) {
            // Create file for this drone
            std::string filename = output_folder_ + "/node_" + std::to_string(drone_id) + ".txt";
            DroneFileHandler handler;
            handler.file_stream.open(filename);

            if (!handler.file_stream.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
                continue;
            }

            drone_files_[drone_id] = std::move(handler);

            // Subscribe to bspline topic
            std::string topic = "/drone_" + std::to_string(drone_id) + "_planning/bspline";
            auto sub = this->create_subscription<traj_utils::msg::Bspline>(
                topic, 10,
                [this, drone_id](const traj_utils::msg::Bspline::SharedPtr msg) {
                    this->bsplineCallback(msg, drone_id);
                });

            subscriptions_[drone_id] = sub;

            RCLCPP_INFO(this->get_logger(), "  Drone %d: subscribed to %s -> %s",
                       drone_id, topic.c_str(), filename.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Multi-Drone Trajectory Recorder started");
        RCLCPP_INFO(this->get_logger(), "  Number of drones: %d", num_drones_);
        RCLCPP_INFO(this->get_logger(), "  Output folder: %s", output_folder_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Sampling dt: %.3f s", sampling_dt_);
    }

    ~MultiDroneTrajectoryRecorder() {
        RCLCPP_INFO(this->get_logger(), "Saving trajectories...");
        for (auto& [drone_id, handler] : drone_files_) {
            if (handler.file_stream.is_open()) {
                handler.file_stream.close();
                RCLCPP_INFO(this->get_logger(), "  Drone %d: %d lines saved",
                           drone_id, handler.line_number - 1);
            }
        }
        RCLCPP_INFO(this->get_logger(), "All trajectories saved to: %s", output_folder_.c_str());
    }

private:
    std::string createOutputFolder(const std::string& base_folder) {
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&time_t);

        // Format: YYYYMMDD_HHMMSS
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        std::string datetime_str = ss.str();

        // Create full path
        std::string full_path = base_folder + "_" + datetime_str;

        try {
            fs::create_directories(full_path);
            RCLCPP_INFO(this->get_logger(), "Created output folder: %s", full_path.c_str());
            return full_path;
        } catch (const fs::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create folder: %s", e.what());
            return "";
        }
    }

    void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg, int drone_id) {
        if (drone_files_.find(drone_id) == drone_files_.end()) {
            RCLCPP_WARN(this->get_logger(), "No file handler for drone %d", drone_id);
            return;
        }

        auto& handler = drone_files_[drone_id];

        if (!handler.file_stream.is_open()) {
            RCLCPP_WARN(this->get_logger(), "File not open for drone %d", drone_id);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Drone %d: trajectory (traj_id: %ld, %zu points)",
                    drone_id, msg->traj_id, msg->pos_pts.size());

        if (msg->pos_pts.empty() || msg->knots.empty()) {
            RCLCPP_WARN(this->get_logger(), "Drone %d: empty trajectory", drone_id);
            return;
        }

        // Convert control points to Eigen
        std::vector<Eigen::Vector3d> ctrl_pts;
        for (const auto& pt : msg->pos_pts) {
            ctrl_pts.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        }

        // Create B-spline evaluator
        SimpleBsplineEvaluator bspline(ctrl_pts, msg->knots, msg->order);
        double duration = bspline.getDuration();

        if (duration <= 0) {
            RCLCPP_WARN(this->get_logger(), "Drone %d: invalid duration %.3f", drone_id, duration);
            return;
        }

        // Sample the trajectory
        rclcpp::Time msg_time(msg->start_time);
        double msg_offset = (msg_time - global_start_time_).seconds();

        for (double t = 0.0; t <= duration; t += sampling_dt_) {
            Eigen::Vector3d pos = bspline.evaluate(t);
            double timestamp = msg_offset + t;
            writeTrajectoryPoint(handler, drone_id, timestamp, pos.x(), pos.y(), pos.z(), 0.0);
        }

        // Always write the final point
        Eigen::Vector3d final_pos = bspline.evaluate(duration);
        writeTrajectoryPoint(handler, drone_id, msg_offset + duration,
                           final_pos.x(), final_pos.y(), final_pos.z(), 0.0);

        handler.file_stream.flush();
    }

    void writeTrajectoryPoint(DroneFileHandler& handler, int drone_id,
                             double timestamp, double x, double y, double z, double yaw) {
        if (!handler.file_stream.is_open()) return;

        // Format: line_number, drone_id, timestamp, move, x, y, z, yaw, r, g, b
        handler.file_stream << handler.line_number++ << ","
                           << drone_id << ","
                           << std::fixed << std::setprecision(2) << timestamp << ","
                           << "move,"
                           << std::setprecision(2) << x << ","
                           << std::setprecision(2) << y << ","
                           << std::setprecision(1) << z << ","
                           << std::setprecision(1) << yaw << ","
                           << default_rgb_[0] << ","
                           << default_rgb_[1] << ","
                           << default_rgb_[2] << "\n";
    }

    std::map<int, rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr> subscriptions_;
    std::map<int, DroneFileHandler> drone_files_;

    std::string output_folder_;
    int num_drones_;
    double sampling_dt_;
    double record_height_;
    std::vector<int> default_rgb_;
    rclcpp::Time global_start_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiDroneTrajectoryRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
