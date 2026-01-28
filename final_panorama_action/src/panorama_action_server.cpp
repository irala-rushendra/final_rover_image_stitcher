#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/stitching.hpp"

// THIS HEADER IS GENERATED FROM YOUR .action FILE
// Note the namespace matches the new package name
#include "final_panorama_action/action/capture_panorama.hpp"

namespace fs = std::filesystem;

class PanoramaActionServer : public rclcpp::Node
{
public:
    using CapturePanorama = final_panorama_action::action::CapturePanorama;
    using GoalHandlePanorama = rclcpp_action::ServerGoalHandle<CapturePanorama>;

    PanoramaActionServer() : Node("panorama_action_server")
    {
        // --- PARAMETERS ---
        this->declare_parameter("divide_images", false); // Your specific flag
        this->declare_parameter("mode", "panorama");
        this->declare_parameter("image_topic", "/image_raw"); // Default for laptop webcam

        std::string img_topic = this->get_parameter("image_topic").as_string();

        // --- SUBSCRIBERS ---
        // 1. Angle Subscriber (Continuous)
        angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/servo_angle", 10,
            std::bind(&PanoramaActionServer::angle_callback, this, std::placeholders::_1));

        // 2. Camera Subscriber (Continuous)
        RCLCPP_INFO(this->get_logger(), "Listening to camera: %s", img_topic.c_str());
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            img_topic, 10,
            std::bind(&PanoramaActionServer::image_callback, this, std::placeholders::_1));

        // --- ACTION SERVER SETUP ---
        this->action_server_ = rclcpp_action::create_server<CapturePanorama>(
            this,
            "capture_panorama",
            std::bind(&PanoramaActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PanoramaActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PanoramaActionServer::handle_accepted, this, std::placeholders::_1));

        current_angle_ = -999.0;
        has_image_ = false;

        RCLCPP_INFO(this->get_logger(), "Action Server Ready. Waiting for goal...");
    }

private:
    rclcpp_action::Server<CapturePanorama>::SharedPtr action_server_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    std::mutex data_mutex_;
    double current_angle_;
    cv::Mat latest_image_;
    bool has_image_;

    const std::string temp_dir_ = "/tmp/panorama_temp/";
    const std::string final_output_ = "final_panorama_stitched.jpg";

    // --- CALLBACKS ---
    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_angle_ = msg->data;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_image_ = cv_ptr->image;
            has_image_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    // --- ACTION LOGIC ---
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CapturePanorama::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received Goal: Sweep %.1f deg", goal->total_angle_sweep);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePanorama> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received Cancel Request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePanorama> goal_handle)
    {
        // Spawns a new thread to execute the long-running task so we don't block the node
        std::thread{std::bind(&PanoramaActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePanorama> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<CapturePanorama::Feedback>();
        auto result = std::make_shared<CapturePanorama::Result>();

        // 1. Initial Checks
        double start_angle = 0.0;
        {
            // Wait briefly for data to arrive if the node just started
            int retries = 0;
            while(rclcpp::ok() && retries < 50) { 
                std::lock_guard<std::mutex> lock(data_mutex_);
                if(current_angle_ != -999.0 && has_image_) {
                    start_angle = current_angle_;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                retries++;
            }
            if(retries >= 50) {
                result->success = false;
                result->message = "Aborted: No angle/camera data on topics.";
                goal_handle->abort(result);
                return;
            }
        }

        // 2. Setup Files
        if (fs::exists(temp_dir_)) fs::remove_all(temp_dir_);
        fs::create_directory(temp_dir_);

        std::vector<std::string> captured_files;
        int steps = static_cast<int>(goal->total_angle_sweep / goal->angle_interval) + 1;
        
        RCLCPP_INFO(this->get_logger(), "Starting Scan. Initial: %.2f deg. Steps: %d", start_angle, steps);

        // 3. CAPTURE LOOP
        for (int i = 0; i < steps; ++i) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Cancelled by user.";
                goal_handle->canceled(result);
                return;
            }

            double target_angle = start_angle + (i * goal->angle_interval);

            // Notify user
            feedback->status = "Moving to " + std::to_string(target_angle);
            feedback->image_count = i;
            goal_handle->publish_feedback(feedback);

            // --- WAIT FOR ANGLE ---
            bool reached = false;
            while (!reached && rclcpp::ok()) {
                if (goal_handle->is_canceling()) break;

                double curr;
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    curr = current_angle_;
                }

                // Tolerance +/- 2 degrees
                if (std::abs(curr - target_angle) <= 2.0) {
                    reached = true;
                } else {
                    // Update feedback with current angle while waiting
                    feedback->current_angle = (float)curr;
                    goal_handle->publish_feedback(feedback);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
            
            // Stabilization delay
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Capture
            cv::Mat temp_img;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                temp_img = latest_image_.clone();
            }

            if (!temp_img.empty()) {
                std::string fname = temp_dir_ + "img_" + std::to_string(i) + ".jpg";
                cv::imwrite(fname, temp_img);
                captured_files.push_back(fname);
                RCLCPP_INFO(this->get_logger(), "Captured Image @ %.2f", target_angle);
            }
        }

        // 4. STITCHING
        feedback->status = "Stitching...";
        goal_handle->publish_feedback(feedback);
        
        cv::Mat final_img = stitch_images_custom(captured_files);

        // 5. FINALIZE
        if (!final_img.empty()) {
            cv::imwrite(final_output_, final_img);
            result->success = true;
            result->message = "Success.";
            result->final_image_path = final_output_;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded. Saved to %s", final_output_.c_str());
        } else {
            result->success = false;
            result->message = "Stitching failed.";
            goal_handle->abort(result);
        }
        
        fs::remove_all(temp_dir_);
    }

    cv::Mat stitch_images_custom(const std::vector<std::string>& file_paths)
    {
        // Retrieve ROS Parameters for config
        bool divide = this->get_parameter("divide_images").as_bool();
        std::string mode_str = this->get_parameter("mode").as_string();

        cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;
        if (mode_str == "scans") mode = cv::Stitcher::SCANS;

        std::vector<cv::Mat> imgs;
        for (const auto& path : file_paths) {
            cv::Mat img = cv::imread(path);
            if (img.empty()) continue;

            if (divide) {
                // --- YOUR CUSTOM SPLIT LOGIC ---
                cv::Rect rect(0, 0, img.cols / 2, img.rows);
                imgs.push_back(img(rect).clone());
                rect.x = img.cols / 3;
                imgs.push_back(img(rect).clone());
                rect.x = img.cols / 2;
                imgs.push_back(img(rect).clone());
            } else {
                imgs.push_back(img);
            }
        }

        if (imgs.empty()) return cv::Mat();

        cv::Mat pano;
        cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode);
        cv::Stitcher::Status status = stitcher->stitch(imgs, pano);

        if (status != cv::Stitcher::OK) {
            RCLCPP_ERROR(this->get_logger(), "Stitching Error Code: %d", int(status));
            return cv::Mat();
        }
        return pano;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PanoramaActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
