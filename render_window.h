#pragma once

#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class RenderWindow : public QWidget
{
    Q_OBJECT

public:
    explicit RenderWindow(QWidget* parent = nullptr);
    void renderPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
    QVTKOpenGLNativeWidget* vtkWidget;
    vtkSmartPointer<vtkRenderer> renderer;
};