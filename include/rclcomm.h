/*
 * @Author: chengyangkj
 * @Date: 2021-10-30 02:15:28
 * @LastEditTime: 2021-12-01 06:03:51
 * @LastEditors: chengyangkj
 * @Description: ros2的通信类 在这个类进行订阅与发布话题
 * @FilePath: /ros2_qt_demo/include/ros2_qt_demo/rclcomm.h
 * https://github.com/chengyangkj
 */
#ifndef RCLCOMM_H
#define RCLCOMM_H
#include <QObject>
#include <QThread>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/float64_multi_array.hpp> 
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rviz_common/visualization_manager.hpp> 
#include <string>
#include <QImage>
class rclcomm:public QThread
{
    Q_OBJECT
public:
    rclcomm();
    std::shared_ptr<rclcpp::Node> node;
    void publish_topic(int count);
    void recv_callback(std_msgs::msg::Int32::SharedPtr msg);
    void topicCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg); 
    void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);
protected:
    void run();
private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _publisher;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _subscription;
    // std::shared_ptr<rclcpp::Node> node;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr topic_subscriber_;
    image_transport::ImageTransport* image_transport;  // 使用指针以便安全初始化
    image_transport::Subscriber image_subscriber_;
    
signals:
    void emitTopicData(QString);
    void topicDataReceived(const std::string& data);
    void imageReceived(const QImage& image);
};
#endif // RCLCOMM_H
