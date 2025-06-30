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
    rectangleSelectionAct = new QAction("矩形绘制(&R)", this);
    rectangleSelectionAct->setStatusTip("启用矩形绘制模式，拖拽鼠标在画布上绘制矩形");
    rectangleSelectionAct->setCheckable(true);
    connect(rectangleSelectionAct, &QAction::triggered, this, &MainWindow::onRectangleSelection);

    // 圆形选择动作
    circleSelectionAct = new QAction("圆形绘制(&C)", this);
    circleSelectionAct->setStatusTip("启用圆形绘制模式，拖拽鼠标在画布上绘制圆形");
    circleSelectionAct->setCheckable(true);
    connect(circleSelectionAct, &QAction::triggered, this, &MainWindow::onCircleSelection);

    // 多边形选择动作
    polygonSelectionAct = new QAction("多边形绘制(&P)", this);
    polygonSelectionAct->setStatusTip("启用多边形绘制模式，左键放置顶点，Backspace撤销，右键完成绘制");
    polygonSelectionAct->setCheckable(true);
    connect(polygonSelectionAct, &QAction::triggered, this, &MainWindow::onPolygonSelection);

    // 清空画布动作
    clearCanvasAct = new QAction("清空画布(&L)", this);
    clearCanvasAct->setStatusTip("清空画布上的所有形状");
    connect(clearCanvasAct, &QAction::triggered, this, &MainWindow::onClearCanvas);

    // 确认选取动作
    confirmSelectionAct = new QAction("确认选取(&S)", this);
    confirmSelectionAct->setStatusTip("根据画布上的形状执行点云选取");
    connect(confirmSelectionAct, &QAction::triggered, this, &MainWindow::onConfirmSelection);

    // 清除选择动作
    clearSelectionAct = new QAction("清除选中点(&C)", this);
    clearSelectionAct->setStatusTip("清除当前选中的点云");
    connect(clearSelectionAct, &QAction::triggered, this, &MainWindow::onClearSelection);
    
    // 矩形转多边形动作
    convertRectangleToPolygonAct = new QAction("矩形转多边形(&T)", this);
    convertRectangleToPolygonAct->setStatusTip("将选中的矩形转换为多边形");
    connect(convertRectangleToPolygonAct, &QAction::triggered, this, &MainWindow::onConvertRectangleToPolygon);
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
    fileToolBar = addToolBar("点云处理工具");
    fileToolBar->addAction(importPLYAct);
    fileToolBar->addSeparator();
    
    // 绘制工具组
    fileToolBar->addAction(rectangleSelectionAct);
    fileToolBar->addAction(circleSelectionAct);
    fileToolBar->addAction(polygonSelectionAct);
    fileToolBar->addSeparator();
    
    // 画布操作组
    fileToolBar->addAction(clearCanvasAct);
    fileToolBar->addAction(confirmSelectionAct);
    fileToolBar->addAction(clearSelectionAct);
    fileToolBar->addAction(convertRectangleToPolygonAct);
    fileToolBar->addSeparator();

    // 添加遮挡检测的复选框
    QCheckBox *occlusionCheckBox = new QCheckBox("启用深度选择", this);
    occlusionCheckBox->setChecked(true); // 默认开启
    occlusionCheckBox->setStatusTip("开启或关闭基于深度的遮挡检测选择模式");
    connect(occlusionCheckBox, &QCheckBox::toggled, this, &MainWindow::onOcclusionDetectionToggled);
    fileToolBar->addWidget(occlusionCheckBox);
    
    // 添加锁定视图的复选框
    QCheckBox *viewLockCheckBox = new QCheckBox("锁定画布", this);
    viewLockCheckBox->setChecked(false); // 默认不锁定
    viewLockCheckBox->setStatusTip("锁定画布视图，禁用旋转和缩放操作");
    connect(viewLockCheckBox, &QCheckBox::toggled, this, &MainWindow::onViewLockToggled);
    fileToolBar->addWidget(viewLockCheckBox);

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
        updateStatusInfo(); // 更新状态信息
    } else {
        QMessageBox::warning(this, "错误", message);
        statusBar()->showMessage("导入失败", 3000);
    }
}

void MainWindow::onRectangleSelection()
{
    bool enabled = rectangleSelectionAct->isChecked();
    
    // 如果启用矩形绘制，则禁用其他绘制模式
    if (enabled) {
        circleSelectionAct->setChecked(false);
        polygonSelectionAct->setChecked(false);
    }
    
    pointCloudViewer->enableRectangleSelection(enabled);
    
    if (enabled) {
        statusBar()->showMessage("矩形绘制模式已启用，拖拽鼠标在画布上绘制矩形", 2000);
    } else {
        statusBar()->showMessage("矩形绘制模式已禁用", 2000);
    }
    
    updateStatusInfo();
}

void MainWindow::onCircleSelection()
{
    bool enabled = circleSelectionAct->isChecked();
    
    // 如果启用圆形绘制，则禁用其他绘制模式
    if (enabled) {
        rectangleSelectionAct->setChecked(false);
        polygonSelectionAct->setChecked(false);
    }
    
    pointCloudViewer->enableCircleSelection(enabled);
    
    if (enabled) {
        statusBar()->showMessage("圆形绘制模式已启用，拖拽鼠标在画布上绘制圆形", 2000);
    } else {
        statusBar()->showMessage("圆形绘制模式已禁用", 2000);
    }
    
    updateStatusInfo();
}

void MainWindow::onPolygonSelection()
{
    bool enabled = polygonSelectionAct->isChecked();
    
    // 如果启用多边形绘制，则禁用其他绘制模式
    if (enabled) {
        rectangleSelectionAct->setChecked(false);
        circleSelectionAct->setChecked(false);
    }
    
    pointCloudViewer->enablePolygonSelection(enabled);
    
    if (enabled) {
        statusBar()->showMessage("多边形绘制模式已启用，左键放置顶点，Backspace撤销，右键完成绘制", 4000);
    } else {
        statusBar()->showMessage("多边形绘制模式已禁用", 2000);
    }
    
    updateStatusInfo();
}

void MainWindow::onClearCanvas()
{
    pointCloudViewer->clearCanvas();
    statusBar()->showMessage("画布已清空", 2000);
    updateStatusInfo();
}

void MainWindow::onConfirmSelection()
{
    size_t shapeCount = pointCloudViewer->getCanvasShapeCount();
    if (shapeCount == 0) {
        statusBar()->showMessage("画布为空，无法执行选取", 2000);
        return;
    }
    
    pointCloudViewer->confirmSelection();
    statusBar()->showMessage(QString("已根据%1个形状执行点云选取，画布已自动清空").arg(shapeCount), 3000);
    updateStatusInfo();
}

void MainWindow::onClearSelection()
{
    pointCloudViewer->clearAllSelectedPoints();
    statusBar()->showMessage("所有选中的点已清除", 2000);
    updateStatusInfo();
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

void MainWindow::onViewLockToggled(bool checked)
{
    pointCloudViewer->enableViewLock(checked);
    if (checked) {
        statusBar()->showMessage("画布已锁定，禁用视图操作", 2000);
    } else {
        statusBar()->showMessage("画布已解锁，启用视图操作", 2000);
    }
}

void MainWindow::onConvertRectangleToPolygon()
{
    bool success = pointCloudViewer->convertRectangleToPolygon();
    if (success) {
        statusBar()->showMessage("矩形已成功转换为多边形", 2000);
    } else {
        statusBar()->showMessage("转换失败：请先选中一个矩形", 2000);
    }
    updateStatusInfo();
}

void MainWindow::updateStatusInfo()
{
    // 更新状态栏信息，显示画布形状数和选中点数
    size_t shapeCount = pointCloudViewer->getCanvasShapeCount();
    size_t selectedCount = pointCloudViewer->getSelectedPointCount();
    
    QString statusText = QString("画布形状: %1 | 选中点数: %2").arg(shapeCount).arg(selectedCount);
    statusBar()->showMessage(statusText, 0); // 0表示永久显示
} 