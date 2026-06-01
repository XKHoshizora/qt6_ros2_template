#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), count_(0) {
    ui->setupUi(this);
    setupConnections();
    initROS();
}

MainWindow::~MainWindow() {
    delete ui;
#ifdef USE_ROS
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
#endif
}

void MainWindow::setupConnections() {
    connect(ui->button_connect, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    
    publishTimer_ = new QTimer(this);
    connect(publishTimer_, &QTimer::timeout, this, &MainWindow::onPublishTimer);
}

void MainWindow::initROS() {
#ifdef USE_ROS
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("qt_ros2_node");
    publisher_ = node_->create_publisher<std_msgs::msg::String>("topic", 10);
#endif
}

void MainWindow::onConnectButtonClicked() {
    if (publishTimer_->isActive()) {
        publishTimer_->stop();
        ui->button_connect->setText("Connect");
        ui->view_logging->addItem("Disconnected from ROS");
    } else {
        publishTimer_->start(500);  // Publish every 500 ms
        ui->button_connect->setText("Disconnect");
        ui->view_logging->addItem("Connected to ROS");
    }
}

void MainWindow::onPublishTimer() {
#ifdef USE_ROS
    if (!rclcpp::ok()) {
        QMessageBox::critical(this, "ROS Error", "ROS is not running!");
        publishTimer_->stop();
        return;
    }

    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2! " + std::to_string(count_++);
    publisher_->publish(message);

    ui->view_logging->addItem(QString::fromStdString("Published: " + message.data));
    rclcpp::spin_some(node_);
#else
    // No ROS at build time: run as a plain Qt app so the dual-build still works.
    ui->view_logging->addItem(QString::fromStdString(
        "Published (no ROS): Hello, ROS 2! " + std::to_string(count_++)));
#endif
}
