/*
 * @Author: chengyangkj
 * @Date: 2021-10-30 02:09:08
 * @LastEditTime: 2021-12-01 06:01:17
 * @LastEditors: chengyangkj
 * @Description: 
 * @FilePath: /ros2_qt_demo/include/ros2_qt_demo/mainwindow.h
 * https://github.com/chengyangkj
 */
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "rclcomm.h"
#include <iostream>

#include <QtWidgets/QMainWindow>
#include <QToolBar>
#include <QStatusBar>
#include <QLabel>  // 用于显示图像
#include <QImage>  // 用于显示图像
#include <QHBoxLayout>   // 水平布局
#include <QVBoxLayout>   // 垂直布局
#include <QWidget>
#include <QVector3D>
#include <QObject>  // 确保QObject头文件被包含

#include <rviz_common/visualization_frame.hpp>  // RViz2头文件
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>  // 关键：RViz 节点抽象接口
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    rclcomm *commNode;

    QToolBar* toolBar;
    QStatusBar* statusBar;
    QLabel* image_label;  // 图像显示标签

    // RViz相关组件
    rviz_common::RenderPanel* rviz_panel;          // RViz渲染面板
    rviz_common::VisualizationManager* rviz_manager; // RViz管理器
    rviz_common::VisualizationFrame* rviz_frame;

    // RViz布局管理器（用于划分界面区域）
    QWidget* central_widget;
    QHBoxLayout* main_layout;    // 主水平布局（分左右两部分）
    QVBoxLayout* left_layout;    // 左侧垂直布局（放原图）
    QVBoxLayout* right_layout;   // 右侧垂直布局（分上下两部分）

public slots:
    void updateTopicInfo(QString);
    void updateStatusBar(const std::string& data);
    void updateImage(const QImage& image);
    void on_actionAbout_triggered();
    void setupCameraPosition();
    void setupSceneLighting();
};
#endif // MAINWINDOW_H
