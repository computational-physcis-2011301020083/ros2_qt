/*
 * @Author: chengyangkj
 * @Date: 2021-10-30 02:08:29
 * @LastEditTime: 2021-12-01 06:18:27
 * @LastEditors: chengyangkj
 * @Description: ros2的通信类 在这个类进行订阅与发布话题
 * @FilePath: /ros2_qt_demo/src/rclcomm.cpp
 * https://github.com/chengyangkj
 */
#include "rclcomm.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>
rclcomm::rclcomm()  {
  int argc=0;
  char **argv=NULL;
  rclcpp::init(argc,argv);
  node=rclcpp::Node::make_shared("ros2_qt_demo");
  _publisher =node->create_publisher<std_msgs::msg::Int32>("ros2_qt_demo_publish", 10);
  _subscription = node->create_subscription<std_msgs::msg::Int32>("ros2_qt_demo_publish", 10,std::bind(&rclcomm::recv_callback,this,std::placeholders::_1));
  topic_subscriber_ = node->create_subscription<std_msgs::msg::Float64MultiArray>("model_sensor",10,std::bind(&rclcomm::topicCallback, this, std::placeholders::_1));
  // 图像订阅
  image_transport::ImageTransport it(node);
  image_subscriber_ = it.subscribe("camera/image_raw",10,std::bind(&rclcomm::imageCallback, this, std::placeholders::_1));
  this->start();
}
void rclcomm::run(){
    //20HZ
    std_msgs::msg::Int32 pub_msg;
    pub_msg.data=0;
    rclcpp::WallRate loop_rate(1);
    while (rclcpp::ok())
    {
        pub_msg.data++;
        _publisher->publish(pub_msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
}
void rclcomm::recv_callback(const std_msgs::msg::Int32::SharedPtr msg){
    //  RCLCPP_INFO(node->get_logger(), "I heard: '%d'", msg->data);
    emitTopicData("I head from ros2_qt_demo_publish:"+QString::fromStdString(std::to_string(msg->data)));
}

void rclcomm::topicCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::stringstream dataStream;
    dataStream << "Sensor State(degree): ";
    dataStream << "Excavator Swing Angle: " << msg->data[0]*180/M_PI << ", ";
    dataStream << "Boom Angle: " << msg->data[1]*180/M_PI << ", ";
    dataStream << "Arm Angle: " << msg->data[2]*180/M_PI << ", ";
    dataStream << "Bucket Angle: " << msg->data[3]*180/M_PI << ".";
    Q_EMIT topicDataReceived(dataStream.str());
}

void rclcomm::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat cv_image = cv_ptr->image;
        QImage q_image(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step, QImage::Format_BGR888);
        Q_EMIT imageReceived(q_image.rgbSwapped());  // BGR转RGB
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    }
}


