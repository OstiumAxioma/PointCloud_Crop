#include "mainwindow.h"
#include <QApplication>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QLabel>
#include <QVTKOpenGLWidget.h>

// VTK头文件
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPLYReader.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkElevationFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("点云处理应用");
    resize(800, 600);

    createActions();
    createMenus();
    createToolBars();
    createStatusBar();
    
    // 先创建一个简单的界面，确保程序能稳定运行
    setupSimpleWidget();
    
    // 延迟初始化VTK组件
    QTimer::singleShot(2000, this, &MainWindow::setupVTKWidget);
}

MainWindow::~MainWindow()
{
}

void MainWindow::createActions()
{
    // 退出动作
    exitAct = new QAction("退出(&Q)", this);
    exitAct->setShortcuts(QKeySequence::Quit);
    exitAct->setStatusTip("退出应用程序");
    connect(exitAct, &QAction::triggered, this, &QWidget::close);

    // 关于动作
    aboutAct = new QAction("关于(&A)", this);
    aboutAct->setStatusTip("显示应用程序的关于对话框");
    connect(aboutAct, &QAction::triggered, [this]() {
        QMessageBox::about(this, "关于 点云处理应用",
                          "这是一个基于VTK和Qt的点云处理应用。");
    });

    // 导入PLY文件动作
    importPLYAct = new QAction("导入PLY(&I)", this);
    importPLYAct->setStatusTip("导入PLY点云文件");
    connect(importPLYAct, &QAction::triggered, this, &MainWindow::importPLY);
}

void MainWindow::createMenus()
{
    fileMenu = menuBar()->addMenu("文件(&F)");
    fileMenu->addAction(importPLYAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    helpMenu = menuBar()->addMenu("帮助(&H)");
    helpMenu->addAction(aboutAct);
}

void MainWindow::createToolBars()
{
    fileToolBar = addToolBar("文件");
    fileToolBar->addAction(importPLYAct);
    fileToolBar->addSeparator();
    fileToolBar->addAction(exitAct);
}

void MainWindow::createStatusBar()
{
    statusBar()->showMessage("就绪");
}

void MainWindow::setupSimpleWidget()
{
    QLabel *label = new QLabel("点云处理应用正在初始化...", this);
    label->setAlignment(Qt::AlignCenter);
    label->setStyleSheet("QLabel { font-size: 18px; color: blue; }");
    setCentralWidget(label);
    statusBar()->showMessage("简单界面创建成功", 2000);
}

void MainWindow::setupVTKWidget()
{
    try {
        statusBar()->showMessage("正在初始化VTK...", 1000);

        // 步骤1：创建QVTKOpenGLWidget
        vtkWidget = new QVTKOpenGLWidget(this);
        setCentralWidget(vtkWidget);

        // 步骤2：创建VTK对象
        renderer = vtkSmartPointer<vtkRenderer>::New();
        renderer->SetBackground(0.0, 0.0, 0.0); // 黑色背景

        // 步骤3：获取QVTKOpenGLWidget的渲染窗口
        renderWindow = vtkWidget->GetRenderWindow();
        renderWindow->AddRenderer(renderer);

        // 步骤4：设置交互器
        renderWindowInteractor = renderWindow->GetInteractor();
        renderWindowInteractor->SetRenderWindow(renderWindow);
        
        // 设置交互样式为轨迹球相机
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        renderWindowInteractor->SetInteractorStyle(style);
        
        renderWindowInteractor->Initialize();

        statusBar()->showMessage("VTK集成到Qt界面成功！", 2000);
    } catch (const std::exception& e) {
        QMessageBox::warning(this, "警告", QString("VTK初始化失败: %1").arg(e.what()));
        statusBar()->showMessage("VTK初始化失败", 3000);
    }
}

void MainWindow::importPLY()
{
    try {
        // 弹出文件选择对话框
        QString fileName = QFileDialog::getOpenFileName(
            this,
            "选择PLY文件",
            "",
            "PLY文件 (*.ply);;所有文件 (*.*)"
        );

        if (fileName.isEmpty()) {
            statusBar()->showMessage("未选择文件", 2000);
            return;
        }

        statusBar()->showMessage("正在加载PLY文件...", 1000);

        // 创建PLY读取器
        plyReader = vtkSmartPointer<vtkPLYReader>::New();
        plyReader->SetFileName(fileName.toStdString().c_str());
        plyReader->Update();

        // 检查文件是否成功读取
        if (plyReader->GetOutput()->GetNumberOfPoints() == 0) {
            QMessageBox::warning(this, "错误", "PLY文件为空或格式不正确");
            statusBar()->showMessage("PLY文件读取失败", 3000);
            return;
        }

        // 创建顶点字形过滤器 - 将点云转换为可渲染的几何体
        glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        glyphFilter->SetInputConnection(plyReader->GetOutputPort());
        glyphFilter->Update();

        // 创建高程过滤器 - 根据Z坐标设置颜色
        elevationFilter = vtkSmartPointer<vtkElevationFilter>::New();
        elevationFilter->SetInputConnection(glyphFilter->GetOutputPort());
        
        // 获取点云的Z轴范围
        double bounds[6];
        plyReader->GetOutput()->GetBounds(bounds);
        double low_z = bounds[4];  // Z轴最小值
        double high_z = bounds[5]; // Z轴最大值
        
        elevationFilter->SetLowPoint(0, 0, low_z);
        elevationFilter->SetHighPoint(0, 0, high_z);

        // 创建映射器
        mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(elevationFilter->GetOutputPort());

        // 创建演员
        actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        // 清除之前的演员
        renderer->RemoveAllViewProps();

        // 添加演员到渲染器
        renderer->AddActor(actor);

        // 重置相机
        renderer->ResetCamera();

        // 刷新渲染窗口
        renderWindow->Render();

        // 显示成功信息
        int numPoints = plyReader->GetOutput()->GetNumberOfPoints();
        int numCells = plyReader->GetOutput()->GetNumberOfCells();
        QString message = QString("PLY文件加载成功！点数: %1, 面片数: %2, Z范围: %3 - %4")
                         .arg(numPoints)
                         .arg(numCells)
                         .arg(low_z, 0, 'f', 2)
                         .arg(high_z, 0, 'f', 2);
        statusBar()->showMessage(message, 3000);

    } catch (const std::exception& e) {
        QMessageBox::warning(this, "错误", QString("导入PLY文件失败: %1").arg(e.what()));
        statusBar()->showMessage("导入PLY文件失败", 3000);
    }
} 