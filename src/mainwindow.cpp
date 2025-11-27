/*
 * @Author: chengyangkj
 * @Date: 2021-10-30 03:11:50
 * @LastEditTime: 2021-12-01 06:19:25
 * @LastEditors: chengyangkj
 * @Description: 程序的主入口类
 * @FilePath: /ros2_qt_demo/src/mainwindow.cpp
 * https://github.com/chengyangkj
 */
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QCloseEvent>
#include <QMessageBox>
#include <QToolBar>
#include <QStatusBar>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <QMetaType>
#include <QVector3D>
#include <QTimer>
#include <cmath>  // 确保已包含，用于 M_PI

#include <OgreCamera.h>
#include <OgreLight.h>
#include <OgreSceneManager.h>
#include <OgreVector3.h>
#include <OgreColourValue.h>
#include <OgreSceneNode.h>

#include <rviz_common/visualization_frame.hpp>  // RViz2头文件
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>  // 关键：RViz 节点抽象接口
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QImage img;
    img.load(":/icon/images/foxy.jpg");
    img.scaled(ui->label->width(),ui->label->height());
    ui->label->setPixmap(QPixmap::fromImage(img).scaled(ui->label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    commNode=new rclcomm();
    connect(commNode,SIGNAL(emitTopicData(QString)),this,SLOT(updateTopicInfo(QString)));

    QObject::connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    setWindowIcon(QIcon(":/images/icon.png"));
    setWindowTitle("TZCO");

    // 注册std::string类型，使其可以在信号槽中传递
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<QImage>("QImage");

    // 初始化中心部件和布局
    central_widget = new QWidget(this);
    setCentralWidget(central_widget);
    main_layout = new QHBoxLayout(central_widget);  // 主布局：左右各占1/2宽度
    // 左侧布局（放原图，占左半部分1/2宽度）
    left_layout = new QVBoxLayout();
    main_layout->addLayout(left_layout, 1);  // 权重1（总宽度的1/2）
    // 右侧布局（分上下两部分，各占1/2高度）
    right_layout = new QVBoxLayout();
    main_layout->addLayout(right_layout, 1);  // 权重1（总宽度的1/2）

    // 创建图像显示标签
    image_label = new QLabel(this);
    image_label->setText("Waiting for image...");
    //image_label->setAlignment(Qt::AlignCenter);
    image_label->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    image_label->setStyleSheet("border: 1px solid black;");
    left_layout->addWidget(image_label, 1);  // 权重1，占左侧部分高度

    // RViz面板（右下角四分之一区域）
    rviz_panel = new rviz_common::RenderPanel();
    right_layout->addStretch(1);  // 上半部分留白（占右侧1/2高度）
    right_layout->addWidget(rviz_panel, 1);  // 下半部分放RViz（占右侧1/2高度）
    
    // 配置RViz
    auto ros_node_abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(commNode->node->get_name());
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_weak = ros_node_abstraction;
    rviz_frame = new rviz_common::VisualizationFrame(rviz_ros_node_weak, this);
    rviz_frame->initialize(rviz_ros_node_weak, QString());
    // 或者指定配置文件名
    // rviz_frame->initialize(rviz_ros_node_weak, "my_rviz_config.rviz");

    /// 方法1：通过VisualizationManager获取DisplayContext
    rviz_manager = rviz_frame->getManager();
    if (rviz_manager) {
        // 在ROS2中，VisualizationManager继承自DisplayContext
        auto display_context = dynamic_cast<rviz_common::DisplayContext*>(rviz_manager);
        if (display_context) {
            rviz_panel->initialize(display_context, false);
        }
    
    // 启动管理器
    rviz_manager->startUpdate();
    }

    // 设置RViz显示属性
    rviz_frame->setSplashPath("");
    rviz_frame->loadDisplayConfig("");
    rviz_frame->setMenuBar(nullptr);
    rviz_frame->setStatusBar(nullptr);
    rviz_frame->setHideButtonVisibility(false);

    // 添加URDF显示
    rviz_common::Display* robot_model_display = rviz_manager->createDisplay("rviz/RobotModel","Robot Model",true);
    if (robot_model_display) {
        robot_model_display->subProp("Robot Description")->setValue("robot_description");
        robot_model_display->subProp("Visual Enabled")->setValue(true);
        rviz_manager->setFixedFrame("base_link");
        
        // 延迟设置相机位置
        QTimer::singleShot(1000, this, SLOT(setupCameraPosition()));
        QTimer::singleShot(1000, this, SLOT(setupSceneLighting()));
    } else {
        RCLCPP_ERROR(commNode->node->get_logger(),"Failed to create RobotModel display!");
    }

    // 创建工具栏
    toolBar = addToolBar("ToolBar");

    // 创建状态栏
    statusBar = new QStatusBar(this);
    setStatusBar(statusBar);

    // 连接信号槽
    QObject::connect(commNode, SIGNAL(topicDataReceived(const std::string&)),this, SLOT(updateStatusBar(const std::string&)));
    QObject::connect(commNode, SIGNAL(imageReceived(const QImage&)),this, SLOT(updateImage(const QImage&)));

    if (commNode->node) {
      toolBar->setEnabled(true);
    }


}
void MainWindow::updateTopicInfo(QString data){
    ui->label_4->clear();
    ui->label_4->setText(data);
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateStatusBar(const std::string& data)
{
    statusBar->showMessage(QString::fromStdString(data));
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>Intelligent Platform</h2><p>Copyright TZCO Robot</p><p>This package is intelligent platform of TZCO.</p>"));
}

// 添加图像更新槽函数
void MainWindow::updateImage(const QImage& image)
{
    // 检查图像是否有效且标签已初始化
    if (image_label && !image.isNull()) {
        // 计算左上角四分之一区域的大小
        int targetWidth = width() / 2;
        int targetHeight = height() / 2;

        // 缩放图像并保持比例
        QImage scaledImage = image.scaled(
            targetWidth, targetHeight,
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        );

        image_label->setPixmap(QPixmap::fromImage(scaledImage));
    }
}

void MainWindow::setupCameraPosition()
{
    if (!rviz_manager) {
        RCLCPP_ERROR(rclcpp::get_logger("MainWindow"), "RViz manager is null!");
        return;
    }

    // 方法1：通过ViewManager获取当前视图的相机
    auto view_manager = rviz_manager->getViewManager();
    if (!view_manager) {
        RCLCPP_ERROR(rclcpp::get_logger("MainWindow"), "Failed to get view manager!");
        return;
    }

    auto current_view = view_manager->getCurrent();
    if (!current_view) {
        RCLCPP_ERROR(rclcpp::get_logger("MainWindow"), "Failed to get current view!");
        return;
    }

    // 方法2：直接设置相机位置
    Ogre::Camera* camera = current_view->getCamera();
    if (camera) {
        // 设置相机位置 (x, y, z)
        camera->setPosition(Ogre::Vector3(0.0f, -11.0f, 11.0f));
        
        // 设置相机看向的点 (x, y, z)
        camera->lookAt(Ogre::Vector3(0.0f, 0.0f, 0.0f));
        
        // 设置相机的朝向（可选）
        camera->setDirection(Ogre::Vector3(0.0f, 1.2f, -1.0f));
        
        // 设置近剪裁平面（可选）
        camera->setNearClipDistance(0.01f);
        
        // 设置远剪裁平面（可选）
        camera->setFarClipDistance(1000.0f);
        
        // 强制刷新渲染
        rviz_manager->queueRender();
        RCLCPP_INFO(rclcpp::get_logger("MainWindow"), "Camera position set successfully");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("MainWindow"), "Failed to get camera from render panel!");
    }
}

void MainWindow::setupSceneLighting()
{
     if (!rviz_manager) {
        RCLCPP_ERROR(rclcpp::get_logger("MainWindow"), "RViz manager is null!");
        return;
    }

    // 获取场景管理器
    Ogre::SceneManager* scene_manager = rviz_manager->getSceneManager();
    if (!scene_manager) {
        RCLCPP_ERROR(rclcpp::get_logger("MainWindow"), "Failed to get scene manager!");
        return;
    }

    // 检查光源是否已存在，如果不存在则创建
    Ogre::Light* ambient_light = nullptr;
    try {
        ambient_light = scene_manager->getLight("MainAmbientLight");
    } catch (const Ogre::ItemIdentityException& e) {
        // 光源不存在，创建新的
        ambient_light = scene_manager->createLight("MainAmbientLight");
        ambient_light->setType(Ogre::Light::LT_DIRECTIONAL);
        RCLCPP_INFO(rclcpp::get_logger("MainWindow"), "Created new ambient light: MainAmbientLight");
    }

    // 设置环境光属性
    ambient_light->setDiffuseColour(Ogre::ColourValue(0.8f, 0.8f, 0.8f));
    ambient_light->setSpecularColour(Ogre::ColourValue(0.9f, 0.9f, 0.9f));
    ambient_light->setDirection(Ogre::Vector3(-1, -1, -1).normalisedCopy());

    // 主方向光 - 使用同样的异常处理方式
    Ogre::Light* main_light = nullptr;
    try {
        main_light = scene_manager->getLight("MainDirectionalLight");
    } catch (const Ogre::ItemIdentityException& e) {
        main_light = scene_manager->createLight("MainDirectionalLight");
        main_light->setType(Ogre::Light::LT_DIRECTIONAL);
        RCLCPP_INFO(rclcpp::get_logger("MainWindow"), "Created new directional light: MainDirectionalLight");
    }

    // 设置主方向光属性
    main_light->setDiffuseColour(Ogre::ColourValue(1.0f, 1.0f, 1.0f));
    main_light->setSpecularColour(Ogre::ColourValue(1.0f, 1.0f, 1.0f));
    //main_light->setDirection(Ogre::Vector3(-0.5f, -1.0f, -0.5f).normalisedCopy());
    main_light->setDirection(Ogre::Vector3(0.f, 1.2f, -1.f).normalisedCopy());


    // 填充光 - 使用同样的异常处理方式
    Ogre::Light* fill_light = nullptr;
    try {
        fill_light = scene_manager->getLight("FillLight");
    } catch (const Ogre::ItemIdentityException& e) {
        fill_light = scene_manager->createLight("FillLight");
        fill_light->setType(Ogre::Light::LT_DIRECTIONAL);
        RCLCPP_INFO(rclcpp::get_logger("MainWindow"), "Created new fill light: FillLight");
    }

    // 设置填充光属性
    fill_light->setDiffuseColour(Ogre::ColourValue(0.4f, 0.4f, 0.6f));
    fill_light->setSpecularColour(Ogre::ColourValue(0.2f, 0.2f, 0.3f));
    fill_light->setDirection(Ogre::Vector3(0.5f, -0.5f, -0.5f).normalisedCopy());

    // 强制刷新渲染
    rviz_manager->queueRender();
    RCLCPP_INFO(rclcpp::get_logger("MainWindow"), "Scene lighting setup complete");
}

