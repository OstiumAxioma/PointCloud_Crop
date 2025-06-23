#include "mainwindow.h"
#include <QApplication>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QLabel>
#include <QCheckBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setWindowTitle("点云处理应用");
    resize(800, 600);

    createActions();
    createMenus();
    createToolBars();
    createStatusBar();
    
    setupUI();
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
    connect(importPLYAct, &QAction::triggered, this, &MainWindow::onImportPLY);

    // 矩形选择动作
    rectangleSelectionAct = new QAction("矩形选择(&R)", this);
    rectangleSelectionAct->setStatusTip("启用矩形选择模式，拖拽鼠标选择点云");
    rectangleSelectionAct->setCheckable(true);
    connect(rectangleSelectionAct, &QAction::triggered, this, &MainWindow::onRectangleSelection);

    // 圆形选择动作
    circleSelectionAct = new QAction("圆形选择(&C)", this);
    circleSelectionAct->setStatusTip("启用圆形选择模式，拖拽鼠标选择点云");
    circleSelectionAct->setCheckable(true);
    connect(circleSelectionAct, &QAction::triggered, this, &MainWindow::onCircleSelection);

    // 清除选择动作
    clearSelectionAct = new QAction("清除选择(&C)", this);
    clearSelectionAct->setStatusTip("清除当前选中的点云");
    connect(clearSelectionAct, &QAction::triggered, this, &MainWindow::onClearSelection);
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
    fileToolBar->addAction(rectangleSelectionAct);
    fileToolBar->addAction(circleSelectionAct);
    fileToolBar->addAction(clearSelectionAct);
    fileToolBar->addSeparator();

    // 添加遮挡检测的复选框
    QCheckBox *occlusionCheckBox = new QCheckBox("启用深度选择", this);
    occlusionCheckBox->setChecked(true); // 默认开启
    occlusionCheckBox->setStatusTip("开启或关闭基于深度的遮挡检测选择模式");
    connect(occlusionCheckBox, &QCheckBox::toggled, this, &MainWindow::onOcclusionDetectionToggled);
    fileToolBar->addWidget(occlusionCheckBox);

    fileToolBar->addSeparator();
    fileToolBar->addAction(exitAct);
}

void MainWindow::createStatusBar()
{
    statusBar()->showMessage("就绪");
}

void MainWindow::setupUI()
{
    // 创建点云查看器
    pointCloudViewer = new PointCloudViewer(this);
    setCentralWidget(pointCloudViewer);
    
    // 初始化VTK
    if (!pointCloudViewer->initializeVTK()) {
        QMessageBox::warning(this, "警告", "VTK初始化失败");
        statusBar()->showMessage("VTK初始化失败", 3000);
    } else {
        statusBar()->showMessage("VTK初始化成功", 2000);
    }
    
    // 创建文件导入器
    fileImporter = new FileImporter(this);
    
    // 连接信号槽
    connect(fileImporter, &FileImporter::fileSelected, 
            this, &MainWindow::onFileSelected);
    connect(fileImporter, &FileImporter::fileSelectionCancelled, 
            this, &MainWindow::onFileSelectionCancelled);
    connect(pointCloudViewer, &PointCloudViewer::importCompleted, 
            this, &MainWindow::onImportCompleted);
}

void MainWindow::onImportPLY()
{
    fileImporter->selectPLYFile(this);
}

void MainWindow::onFileSelected(const QString &fileName)
{
    statusBar()->showMessage("正在加载PLY文件...", 1000);
    pointCloudViewer->importPLYFile(fileName);
}

void MainWindow::onFileSelectionCancelled()
{
    statusBar()->showMessage("未选择文件", 2000);
}

void MainWindow::onImportCompleted(bool success, const QString &message)
{
    if (success) {
        statusBar()->showMessage(message, 3000);
    } else {
        QMessageBox::warning(this, "错误", message);
        statusBar()->showMessage("导入失败", 3000);
    }
}

void MainWindow::onRectangleSelection()
{
    bool enabled = rectangleSelectionAct->isChecked();
    
    // 如果启用矩形选择，则禁用圆形选择
    if (enabled) {
        circleSelectionAct->setChecked(false);
    }
    
    pointCloudViewer->enableRectangleSelection(enabled);
    
    if (enabled) {
        statusBar()->showMessage("矩形选择模式已启用，拖拽鼠标选择点云", 2000);
    } else {
        statusBar()->showMessage("矩形选择模式已禁用", 2000);
    }
}

void MainWindow::onCircleSelection()
{
    bool enabled = circleSelectionAct->isChecked();
    
    // 如果启用圆形选择，则禁用矩形选择
    if (enabled) {
        rectangleSelectionAct->setChecked(false);
    }
    
    pointCloudViewer->enableCircleSelection(enabled);
    
    if (enabled) {
        statusBar()->showMessage("圆形选择模式已启用，拖拽鼠标选择点云", 2000);
    } else {
        statusBar()->showMessage("圆形选择模式已禁用", 2000);
    }
}

void MainWindow::onClearSelection()
{
    pointCloudViewer->clearAllSelectedPoints();
    statusBar()->showMessage("所有选中的点已清除", 2000);
}

void MainWindow::onOcclusionDetectionToggled(bool checked)
{
    pointCloudViewer->enableOcclusionDetection(checked);
    if (checked) {
        statusBar()->showMessage("深度选择已启用", 2000);
    } else {
        statusBar()->showMessage("深度选择已禁用", 2000);
    }
} 