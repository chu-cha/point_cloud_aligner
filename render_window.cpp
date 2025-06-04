#include "render_window.h"
#include <pcl/io/ply_io.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <QVBoxLayout>

RenderWindow::RenderWindow(QWidget* parent) : QWidget(parent)
{
    // ��������� ��������� ����
    setWindowTitle("3D Viewer");
    resize(800, 600);

    // ������� VTK ������
    vtkWidget = new QVTKOpenGLNativeWidget(this);

    // ��������� layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(vtkWidget);
    setLayout(layout);

    // ������������� ���������
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkWidget->setRenderWindow(renderWindow);

    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(renderer);
}

void RenderWindow::renderPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // ������� ���������� ������
    renderer->RemoveAllViewProps();

    // ��������� ������ �� ��������� ����
    pcl::io::savePLYFileBinary("temp.ply", *cloud);

    // ��������� ������ ����� VTK
    vtkNew<vtkPLYReader> reader;
    reader->SetFileName("temp.ply");
    reader->Update();

    // ������� ������������
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);

    // ��������� � �����
    renderer->AddActor(actor);
    renderer->SetBackground(0.1, 0.2, 0.4);
    renderer->ResetCamera();

    // ��������� �����������
    vtkWidget->renderWindow()->Render();

    // ������� ��������� ����
    std::remove("temp.ply");
}