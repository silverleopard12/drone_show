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

// B-spline evaluator using de Boor's algorithm
class SimpleBsplineEvaluator {
public:
    SimpleBsplineEvaluator(const std::vector<Eigen::Vector3d>& ctrl_pts,
                          const std::vector<double>& knots, int order)
        : control_points_(ctrl_pts), knots_(knots), p_(order),
          n_(ctrl_pts.size() - 1), m_(knots.size() - 1) {}

    Eigen::Vector3d evaluateDeBoor(double u) {
        // Clamp u to valid range
        double u_min = knots_[p_];
        double u_max = knots_[m_ - p_];
        double ub = std::min(std::max(u_min, u), u_max);

        // Find knot span k such that u is in [u_k, u_{k+1})
        int k = p_;
        while (k < m_ - p_) {
            if (knots_[k + 1] >= ub)
                break;
            ++k;
        }

        // de Boor's algorithm
        std::vector<Eigen::Vector3d> d;
        for (int i = 0; i <= p_; ++i) {
            int idx = k - p_ + i;
            if (idx >= 0 && idx < static_cast<int>(control_points_.size())) {
                d.push_back(control_points_[idx]);
            } else {
                d.push_back(Eigen::Vector3d::Zero());
            }
        }

        // Recursive computation
        for (int r = 1; r <= p_; ++r) {
            for (int i = p_; i >= r; --i) {
                int idx_alpha = i + k - p_;

                double denominator = knots_[idx_alpha + 1 + p_ - r] - knots_[idx_alpha];
                double alpha = 0.0;
                if (std::abs(denominator) > 1e-10) {
                    alpha = (ub - knots_[idx_alpha]) / denominator;
                }

                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
            }
        }

        return d[p_];
    }

    Eigen::Vector3d evaluate(double t) {
        // Convert time t to parameter u (matches evaluateDeBoorT in C++)
        double u = t + knots_[p_];
        return evaluateDeBoor(u);
    }

    double getDuration() {
        if (knots_.size() <= static_cast<size_t>(p_)) return 0.0;
        return knots_[m_ - p_] - knots_[p_];
    }

private:
    std::vector<Eigen::Vector3d> control_points_;
    std::vector<double> knots_;
    int p_;  // degree (order)
    int n_;  // number of control points - 1
    int m_;  // number of knots - 1
};

// Per-drone file handler
struct DroneFileHandler {
    std::ofstream file_stream;
    int line_number = 1;
    double last_timestamp = 0.0;
    Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
    std::string filename;

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

        // Subscribe to all drones (0-indexed)
        for (int drone_id = 0; drone_id < num_drones_; ++drone_id) {
            // Create file for this drone (node_1.txt instead of node_0.txt)
            std::string filename = output_folder_ + "/node_" + std::to_string(drone_id + 1) + ".txt";
            DroneFileHandler handler;
            handler.filename = filename;
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

        // First, close all files and find max line count
        int max_lines = 0;
        for (auto& [drone_id, handler] : drone_files_) {
            if (handler.file_stream.is_open()) {
                handler.file_stream.close();
                max_lines = std::max(max_lines, handler.line_number - 1);
                RCLCPP_INFO(this->get_logger(), "  Drone %d: %d lines",
                           drone_id, handler.line_number - 1);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Max lines: %d", max_lines);

        // Pad all files to max length with hovering
        if (max_lines > 0) {
            RCLCPP_INFO(this->get_logger(), "Padding all trajectories to %d lines...", max_lines);
            padAllTrajectories(max_lines);
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
            handler.last_timestamp = timestamp;
            handler.last_position = pos;
        }

        // Always write the final point
        Eigen::Vector3d final_pos = bspline.evaluate(duration);
        double final_timestamp = msg_offset + duration;
        writeTrajectoryPoint(handler, drone_id, final_timestamp,
                           final_pos.x(), final_pos.y(), final_pos.z(), 0.0);
        handler.last_timestamp = final_timestamp;
        handler.last_position = final_pos;

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

    void padAllTrajectories(int max_lines) {
        for (auto& [drone_id, handler] : drone_files_) {
            int current_lines = handler.line_number - 1;

            if (current_lines >= max_lines) {
                RCLCPP_INFO(this->get_logger(), "  Drone %d: already at max length", drone_id);
                continue;
            }

            int lines_to_add = max_lines - current_lines;

            // Reopen file in append mode
            std::ofstream file(handler.filename, std::ios::app);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "  Drone %d: failed to reopen file for padding", drone_id);
                continue;
            }

            // Pad with hovering at last position
            double current_timestamp = handler.last_timestamp;
            int current_line_num = current_lines;

            for (int i = 0; i < lines_to_add; ++i) {
                current_line_num++;
                current_timestamp += sampling_dt_;

                // Format: line_number, drone_id, timestamp, move, x, y, z, yaw, r, g, b
                file << current_line_num << ","
                     << drone_id << ","
                     << std::fixed << std::setprecision(2) << current_timestamp << ","
                     << "move,"
                     << std::setprecision(2) << handler.last_position.x() << ","
                     << std::setprecision(2) << handler.last_position.y() << ","
                     << std::setprecision(1) << handler.last_position.z() << ","
                     << std::setprecision(1) << 0.0 << ","
                     << default_rgb_[0] << ","
                     << default_rgb_[1] << ","
                     << default_rgb_[2] << "\n";
            }

            file.close();
            RCLCPP_INFO(this->get_logger(), "  Drone %d: padded %d lines (hover at %.2f, %.2f, %.1f)",
                       drone_id, lines_to_add,
                       handler.last_position.x(), handler.last_position.y(), handler.last_position.z());
        }
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
