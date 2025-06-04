#include "render_window.h"
#include <pcl/io/ply_io.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <QVBoxLayout>

RenderWindow::RenderWindow(QWidget* parent) : QWidget(parent)
{
    // Настройка основного окна
    setWindowTitle("3D Viewer");
    resize(800, 600);

    // Создаем VTK виджет
    vtkWidget = new QVTKOpenGLNativeWidget(this);

    // Настройка layout
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(vtkWidget);
    setLayout(layout);

    // Инициализация рендерера
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkWidget->setRenderWindow(renderWindow);

    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(renderer);
}

void RenderWindow::renderPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Очищаем предыдущие данные
    renderer->RemoveAllViewProps();

    // Сохраняем облако во временный файл
    pcl::io::savePLYFileBinary("temp.ply", *cloud);

    // Загружаем данные через VTK
    vtkNew<vtkPLYReader> reader;
    reader->SetFileName("temp.ply");
    reader->Update();

    // Создаем визуализацию
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);

    // Добавляем в сцену
    renderer->AddActor(actor);
    renderer->SetBackground(0.1, 0.2, 0.4);
    renderer->ResetCamera();

    // Обновляем отображение
    vtkWidget->renderWindow()->Render();

    // Удаляем временный файл
    std::remove("temp.ply");
}