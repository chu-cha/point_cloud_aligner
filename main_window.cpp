#include "main_window.h"

#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <pcl/io/ply_io.h>
#include <vtkRenderWindow.h>


MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
   // , ui(new Ui::MainWindow)
    , cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
   // ui->setupUi(this);

    // Создаем кнопки
    QPushButton* loadButton = new QPushButton("Load PLY", this);
    QPushButton* visualizeButton = new QPushButton("Visualize", this);

    // Размещаем кнопки в layout
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(loadButton);
    layout->addWidget(visualizeButton);

    QWidget* central = new QWidget;
    central->setLayout(layout);
    setCentralWidget(central);

    // Подключаем сигналы
    connect(loadButton, &QPushButton::clicked, this, &MainWindow::onLoadButtonClicked);
    connect(visualizeButton, &QPushButton::clicked, this, &MainWindow::onVisualizeButtonClicked);

    // Инициализируем PCL Visualizer
    //viewer.reset(new pcl::visualization::PCLVisualizer("Point Cloud Viewer", false));
}

MainWindow::~MainWindow()
{
   // delete ui;
}

void MainWindow::onLoadButtonClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        "Open PLY File", "", "PLY Files (*.ply)");

    if (fileName.isEmpty())
        return;

    // Загрузка облака точек через PCL
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName.toStdString(), *cloud) == -1) {
        QMessageBox::critical(this, "Error", "Failed to load PLY file!");
        return;
    }

   /* QMessageBox::information(this, "Success",
        QString("Loaded %1 points").arg(cloud->size()));*/
}

void MainWindow::onVisualizeButtonClicked()
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", "No point cloud loaded!");
        return;
    }

    // Создаем отдельное окно для визуализации
    QWidget* renderWindow = new QWidget();
    renderWindow->setWindowTitle("3D Point Cloud Viewer");
    renderWindow->resize(800, 600);

    // Создаем VTK виджет с правильным типом рендер-окна
    QVTKOpenGLNativeWidget* vtkWidget = new QVTKOpenGLNativeWidget(renderWindow);
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindowVTK;
    vtkWidget->setRenderWindow(renderWindowVTK);

    // Инициализируем PCL Visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("viewer", false));
    viewer->setBackgroundColor(0.1, 0.2, 0.4);

    // Добавляем облако точек
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->resetCamera();

    // Подключаем визуализатор к VTK виджету
    vtkWidget->renderWindow()->AddRenderer(viewer->getRendererCollection()->GetFirstRenderer());

    // Настраиваем layout
    QVBoxLayout* layout = new QVBoxLayout(renderWindow);
    layout->addWidget(vtkWidget);
    renderWindow->setLayout(layout);

    // Устанавливаем интерактор
    viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());

    // Показываем окно
    renderWindow->show();
}