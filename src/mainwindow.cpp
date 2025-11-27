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
    , commNode(nullptr)
    , toolBar(nullptr)
    , statusBar(nullptr)
    , image_label(nullptr)
    , rviz_panel(nullptr)
    , rviz_manager(nullptr)
    , rviz_frame(nullptr)
    , central_widget(nullptr)
    , main_layout(nullptr)
    , left_layout(nullptr)
    , right_layout(nullptr)
{
    ui->setupUi(this);
    
    // 先初始化ROS节点
    commNode = new rclcomm();
    connect(commNode, SIGNAL(emitTopicData(QString)), this, SLOT(updateTopicInfo(QString)));

    connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    setWindowIcon(QIcon(":/images/icon.png"));
    setWindowTitle("TZCO");

    // 注册std::string类型，使其可以在信号槽中传递
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<QImage>("QImage");

    // 初始化中心部件和布局
    central_widget = new QWidget(this);
    setCentralWidget(central_widget);
    main_layout = new QHBoxLayout(central_widget);
    left_layout = new QVBoxLayout();
    right_layout = new QVBoxLayout();
    
    main_layout->addLayout(left_layout, 1);
    main_layout->addLayout(right_layout, 1);

    // 创建图像显示标签
    image_label = new QLabel(this);
    image_label->setText("Waiting for image...");
    image_label->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    image_label->setStyleSheet("border: 1px solid black;");
    left_layout->addWidget(image_label, 1);

    // RViz面板
    rviz_panel = new rviz_common::RenderPanel();
    right_layout->addStretch(1);
    right_layout->addWidget(rviz_panel, 1);
    
    // 创建工具栏和状态栏
    toolBar = addToolBar("ToolBar");
    statusBar = new QStatusBar(this);
    setStatusBar(statusBar);

    // 连接信号槽
    QObject::connect(commNode, SIGNAL(topicDataReceived(const std::string&)), this, SLOT(updateStatusBar(const std::string&)));
    QObject::connect(commNode, SIGNAL(imageReceived(const QImage&)), this, SLOT(updateImage(const QImage&)));

    if (commNode->node) {
        toolBar->setEnabled(true);
    }

    // 延迟初始化RViz - 使用更长的延迟确保窗口完全显示
    QTimer::singleShot(1000, this, &MainWindow::initializeRviz);
}

void MainWindow::initializeRviz()
{
    RCLCPP_INFO(commNode->node->get_logger(), "Starting RViz initialization...");
    
    try {
        // 检查必要的组件是否已创建
        if (!rviz_panel) {
            RCLCPP_ERROR(commNode->node->get_logger(), "RViz panel is null!");
            return;
        }

        if (!commNode || !commNode->node) {
            RCLCPP_ERROR(commNode->node->get_logger(), "ROS node is not available!");
            return;
        }

        // 创建ROS节点抽象
        auto ros_node_abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
            commNode->node->get_name());
        
        rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_weak = ros_node_abstraction;
        
        // 创建可视化管理器
        rviz_common::WindowManagerInterface* wm = nullptr;
        auto clock = ros_node_abstraction->get_raw_node()->get_clock();
        
        RCLCPP_INFO(commNode->node->get_logger(), "Creating VisualizationManager...");
        rviz_manager = new rviz_common::VisualizationManager(rviz_panel, rviz_ros_node_weak, wm, clock);
        
        RCLCPP_INFO(commNode->node->get_logger(), "Initializing RenderPanel...");
        // 正确初始化RenderPanel
        rviz_panel->initialize(rviz_manager, false);
        
        // 设置固定帧
        rviz_manager->setFixedFrame("base_link");
        
        RCLCPP_INFO(commNode->node->get_logger(), "Starting RViz update...");
        // 启动管理器
        rviz_manager->startUpdate();
        
        // 添加URDF显示
        RCLCPP_INFO(commNode->node->get_logger(), "Creating RobotModel display...");
        rviz_common::Display* robot_model_display = rviz_manager->createDisplay(
            "rviz_default_plugins/RobotModel", "Robot Model", true);
            
        if (robot_model_display) {
            robot_model_display->subProp("Robot Description")->setValue("robot_description");
            robot_model_display->subProp("Visual Enabled")->setValue(true);
            
            RCLCPP_INFO(commNode->node->get_logger(), "RobotModel display created successfully");
            
            // 延迟设置相机和光照 - 使用更长的延迟
            QTimer::singleShot(2000, this, SLOT(setupCameraPosition()));
            QTimer::singleShot(2000, this, SLOT(setupSceneLighting()));
        } else {
            RCLCPP_ERROR(commNode->node->get_logger(), "Failed to create RobotModel display!");
        }
        
        RCLCPP_INFO(commNode->node->get_logger(), "RViz initialized successfully");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(commNode->node->get_logger(), "RViz initialization failed: %s", e.what());
        QMessageBox::warning(this, "RViz Initialization Error", 
                            QString("Failed to initialize RViz: %1").arg(e.what()));
    }
}

void MainWindow::updateTopicInfo(QString data)
{
    ui->label_4->clear();
    ui->label_4->setText(data);
}

MainWindow::~MainWindow()
{
    RCLCPP_INFO(commNode->node->get_logger(), "Cleaning up MainWindow...");
    
    // 先停止RViz更新
    if (rviz_manager) {
        rviz_manager->stopUpdate();
    }
    
    // 注意：不要手动删除rviz_manager和rviz_panel
    // 它们有父子关系，会自动清理
    
    if (commNode) {
        delete commNode;
        commNode = nullptr;
    }
    
    delete ui;
}

void MainWindow::updateStatusBar(const std::string& data)
{
    if (statusBar) {
        statusBar->showMessage(QString::fromStdString(data));
    }
}

void MainWindow::updateImage(const QImage& image)
{
    if (image_label && !image.isNull()) {
        int targetWidth = width() / 2;
        int targetHeight = height() / 2;

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
    RCLCPP_INFO(commNode->node->get_logger(), "Setting up camera position...");
    
    if (!rviz_manager) {
        RCLCPP_WARN(commNode->node->get_logger(), "RViz manager not ready for camera setup");
        return;
    }

    auto view_manager = rviz_manager->getViewManager();
    if (!view_manager) {
        RCLCPP_ERROR(commNode->node->get_logger(), "Failed to get view manager!");
        return;
    }

    auto current_view = view_manager->getCurrent();
    if (!current_view) {
        RCLCPP_ERROR(commNode->node->get_logger(), "Failed to get current view!");
        return;
    }

    Ogre::Camera* camera = current_view->getCamera();
    if (camera) {
        camera->setPosition(Ogre::Vector3(0.0f, -11.0f, 11.0f));
        camera->lookAt(Ogre::Vector3(0.0f, 0.0f, 0.0f));
        camera->setDirection(Ogre::Vector3(0.0f, 1.2f, -1.0f));
        camera->setNearClipDistance(0.01f);
        camera->setFarClipDistance(1000.0f);
        
        rviz_manager->queueRender();
        RCLCPP_INFO(commNode->node->get_logger(), "Camera position set successfully");
    } else {
        RCLCPP_ERROR(commNode->node->get_logger(), "Failed to get camera!");
    }
}

void MainWindow::setupSceneLighting()
{
    RCLCPP_INFO(commNode->node->get_logger(), "Setting up scene lighting...");
    
    if (!rviz_manager) {
        RCLCPP_ERROR(commNode->node->get_logger(), "RViz manager is null!");
        return;
    }

    Ogre::SceneManager* scene_manager = rviz_manager->getSceneManager();
    if (!scene_manager) {
        RCLCPP_ERROR(commNode->node->get_logger(), "Failed to get scene manager!");
        return;
    }

    // 简化光照设置，只设置必要的环境光
    try {
        Ogre::Light* ambient_light = scene_manager->createLight("MainAmbientLight");
        ambient_light->setType(Ogre::Light::LT_DIRECTIONAL);
        ambient_light->setDiffuseColour(Ogre::ColourValue(0.8f, 0.8f, 0.8f));
        ambient_light->setSpecularColour(Ogre::ColourValue(0.9f, 0.9f, 0.9f));
        ambient_light->setDirection(Ogre::Vector3(-1, -1, -1).normalisedCopy());
        
        rviz_manager->queueRender();
        RCLCPP_INFO(commNode->node->get_logger(), "Scene lighting setup complete");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(commNode->node->get_logger(), "Failed to setup lighting: %s", e.what());
    }
}

void MainWindow::on_actionAbout_Qt_triggered()
{
    QMessageBox::about(this, tr("About ..."), 
                      tr("<h2>Intelligent Platform</h2><p>Copyright TZCO Robot</p><p>This package is intelligent platform of TZCO.</p>"));
}