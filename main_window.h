#pragma once

#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "render_window.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onLoadButtonClicked();
    void onVisualizeButtonClicked();

private:
    // Ui::MainWindow* ui;  // Remove this line
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};