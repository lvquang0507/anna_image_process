#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>

#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std::chrono_literals;

class LineImageProcessing : public rclcpp::Node
{
public:
    LineImageProcessing() : Node("line_image_processing")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(this->IMAGE_TOPIC_SUB, 10, std::bind(&LineImageProcessing::image_process_callback, this, std::placeholders::_1));
        processed_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(this->IMAGE_TOPIC_PUB, 10);
    }

private:
    void image_process_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image to OpenCV Images
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image_raw_cv = cv_ptr->image;

        // Image processing
        cv::Mat thresholded_image = image_process(image_raw_cv);

        // Convert to ROS image
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, thresholded_image);
        sensor_msgs::msg::Image img_out;
        img_bridge.toImageMsg(img_out);

        processed_img_pub_->publish(img_out);
    }

    cv::Mat image_process(const cv::Mat img)
    {
        cv::Mat dst;
        const double THRESHOLD = 120;
        cv::cvtColor(img, dst, CV_BGR2GRAY);
        cv::threshold(dst, dst, THRESHOLD, 255, cv::THRESH_BINARY);
        return dst;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_img_pub_;
    const std::string IMAGE_TOPIC_SUB = "anna/main_camera/image_raw";
    const std::string IMAGE_TOPIC_PUB = "anna/main_camera/processed_image";
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineImageProcessing>());
    rclcpp::shutdown();
    return 0;
}