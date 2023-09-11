#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
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
        masked_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(this->MASKED_IMAGE_TOPIC_PUB, 10);
        processed_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(this->PROCESSED_IMAGE_TOPIC_PUB, 10);
        centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>(this->CENTROID_TOPIC_PUB, 10);
    }

private:
    void image_process_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image to OpenCV Images
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image_raw_cv = cv_ptr->image;

        // Image processing
        cv::Mat masked_image = image_process(image_raw_cv)[0];
        cv::Mat processed_image = image_process(image_raw_cv)[1];

        // Convert to ROS image
        sensor_msgs::msg::Image::SharedPtr masked_image_out_ = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, masked_image).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr processed_image_out_ = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, processed_image).toImageMsg();

        masked_img_pub_->publish(*masked_image_out_.get());
        processed_img_pub_->publish(*processed_image_out_.get());
    }

    std::vector<cv::Mat> image_process(const cv::Mat img)
    {
        cv::Mat dst;
        cv::Mat src_clone = img.clone();
        const double THRESHOLD = 120;
        cv::cvtColor(img, dst, CV_BGR2GRAY);
        cv::threshold(dst, dst, THRESHOLD, 255, cv::THRESH_BINARY_INV);

        int width = dst.cols;
        int height = dst.rows;

        int search_top = height * 0.5;

        // Remove the upper half of the image (set value = 0)
        for (int i = 0; i < search_top; i++)
        {
            for (int j = 0; j < width; j++)
            {
                dst.at<uchar>(i, j) = 0;
            }
        }
        cv::Moments mu;
        mu = cv::moments(dst);
        cv::Point2f mc;
        int mx = static_cast<int>(mu.m10 / (mu.m00 + 1e-5));
        int my = static_cast<int>(mu.m01 / (mu.m00 + 1e-5));
        mc = cv::Point2d(mx, my);

        this->point_.set__x(mx);
        this->point_.set__y(my);
        this->point_.set__z(0);

        centroid_pub_->publish(this->point_);

        cv::circle(src_clone, mc, 10, cv::Scalar(30, 144, 255), -1);

        std::vector<cv::Mat> image_list;
        image_list.push_back(dst);
        image_list.push_back(src_clone);

        return image_list;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr masked_img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_img_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_pub_;

    geometry_msgs::msg::Point point_;

    const std::string IMAGE_TOPIC_SUB = "anna/main_camera/image_raw";
    const std::string MASKED_IMAGE_TOPIC_PUB = "anna/main_camera/masked_image";
    const std::string PROCESSED_IMAGE_TOPIC_PUB = "anna/main_camera/processed_image";
    const std::string CENTROID_TOPIC_PUB = "anna/centroid_point";
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineImageProcessing>());
    rclcpp::shutdown();
    return 0;
}