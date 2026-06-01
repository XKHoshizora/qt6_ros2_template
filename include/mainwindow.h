#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QListWidget>
#include <QMessageBox>
#include <memory>

#ifdef USE_ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onConnectButtonClicked();
    void onPublishTimer();

private:
    Ui::MainWindow *ui;
#ifdef USE_ROS
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
#endif
    QTimer *publishTimer_;
    size_t count_;

    void initROS();
    void setupConnections();
};

#endif // MAINWINDOW_H
