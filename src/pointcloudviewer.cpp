#include "pointcloudviewer.h"
#include <QMessageBox>
#include <QDebug>

PointCloudViewer::PointCloudViewer(QWidget *parent)
    : QVTKOpenGLWidget(parent)
    , pointCount(0)
    , cellCount(0)
    , zRangeMin(0.0)
    , zRangeMax(0.0)
{
}

PointCloudViewer::~PointCloudViewer()
{
}

bool PointCloudViewer::initializeVTK()
{
    try {
        // 创建VTK对象
        renderer = vtkSmartPointer<vtkRenderer>::New();
        renderer->SetBackground(0.0, 0.0, 0.0); // 黑色背景

        // 获取QVTKOpenGLWidget的渲染窗口
        renderWindow = this->GetRenderWindow();
        renderWindow->AddRenderer(renderer);

        // 设置交互器
        renderWindowInteractor = renderWindow->GetInteractor();
        renderWindowInteractor->SetRenderWindow(renderWindow);
        
        // 创建矩形选择器
        rectangleSelector = vtkSmartPointer<Selector>::New();
        rectangleSelector->SetRenderer(renderer);
        
        // 设置光标回调函数
        rectangleSelector->SetCursorCallback([this](Qt::CursorShape cursor) {
            this->setCursor(cursor);
        });
        
        // 设置交互样式为矩形选择器
        renderWindowInteractor->SetInteractorStyle(rectangleSelector);
        
        renderWindowInteractor->Initialize();

        qDebug() << "VTK初始化成功";
        return true;
    } catch (const std::exception& e) {
        qDebug() << "VTK初始化失败:" << e.what();
        return false;
    }
}

bool PointCloudViewer::importPLYFile(const QString &fileName)
{
    try {
        // 创建PLY读取器
        plyReader = vtkSmartPointer<vtkPLYReader>::New();
        plyReader->SetFileName(fileName.toStdString().c_str());
        plyReader->Update();

        // 检查文件是否成功读取
        if (plyReader->GetOutput()->GetNumberOfPoints() == 0) {
            emit importCompleted(false, "PLY文件为空或格式不正确");
            return false;
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
        zRangeMin = bounds[4];  // Z轴最小值
        zRangeMax = bounds[5]; // Z轴最大值
        
        elevationFilter->SetLowPoint(0, 0, zRangeMin);
        elevationFilter->SetHighPoint(0, 0, zRangeMax);

        // 创建映射器
        mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(elevationFilter->GetOutputPort());

        // 创建演员
        actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        // 清除之前的点云演员（保留选择框）
        if (actor) {
            renderer->RemoveActor(actor);
        }

        // 添加演员到渲染器
        renderer->AddActor(actor);

        // 重置相机
        renderer->ResetCamera();

        // 刷新渲染窗口
        renderWindow->Render();

        // 更新点云信息
        pointCount = plyReader->GetOutput()->GetNumberOfPoints();
        cellCount = plyReader->GetOutput()->GetNumberOfCells();

        // 设置矩形选择器的点云数据（使用带有颜色数据的elevationFilter输出）
        if (rectangleSelector) {
            vtkPolyData* coloredPointData = vtkPolyData::SafeDownCast(elevationFilter->GetOutput());
            if (coloredPointData) {
                rectangleSelector->SetPointCloudData(coloredPointData);
            } else {
                qDebug() << "无法获取带颜色的点云数据";
            }
        }

        QString message = QString("PLY文件加载成功！点数: %1, 面片数: %2, Z范围: %3 - %4")
                         .arg(pointCount)
                         .arg(cellCount)
                         .arg(zRangeMin, 0, 'f', 2)
                         .arg(zRangeMax, 0, 'f', 2);

        emit importCompleted(true, message);
        return true;

    } catch (const std::exception& e) {
        QString errorMsg = QString("导入PLY文件失败: %1").arg(e.what());
        emit importCompleted(false, errorMsg);
        return false;
    }
}

int PointCloudViewer::getPointCount() const
{
    return pointCount;
}

int PointCloudViewer::getCellCount() const
{
    return cellCount;
}

double PointCloudViewer::getZRangeMin() const
{
    return zRangeMin;
}

double PointCloudViewer::getZRangeMax() const
{
    return zRangeMax;
}

void PointCloudViewer::clearDisplay()
{
    renderer->RemoveAllViewProps();
    renderWindow->Render();
    
    pointCount = 0;
    cellCount = 0;
    zRangeMin = 0.0;
    zRangeMax = 0.0;
}
// 下面的方法用于切换选择框
void PointCloudViewer::enableRectangleSelection(bool enable)
{
    if (rectangleSelector) {
        rectangleSelector->SetSelectionShape(SelectionShape::Rectangle);
        rectangleSelector->EnableRectangleSelection(enable);
        
        // 设置光标
        if (enable) {
            this->setCursor(Qt::CrossCursor);
        } else {
            this->setCursor(Qt::ArrowCursor);
        }
    }
}

void PointCloudViewer::enableCircleSelection(bool enable)
{
    if (rectangleSelector) {
        rectangleSelector->SetSelectionShape(SelectionShape::Circle);
        rectangleSelector->EnableRectangleSelection(enable);
        
        // 设置光标
        if (enable) {
            this->setCursor(Qt::CrossCursor);
        } else {
            this->setCursor(Qt::ArrowCursor);
        }
    }
}

void PointCloudViewer::enablePolygonSelection(bool enable)
{
    if (rectangleSelector) {
        rectangleSelector->SetSelectionShape(SelectionShape::Polygon);
        rectangleSelector->EnableRectangleSelection(enable);
        
        // 设置光标
        if (enable) {
            this->setCursor(Qt::CrossCursor);
        } else {
            this->setCursor(Qt::ArrowCursor);
        }
    }
}

void PointCloudViewer::clearAllSelectedPoints()
{
    if (rectangleSelector) {
        rectangleSelector->ClearAllSelectedPoints();
    }
}

void PointCloudViewer::enableOcclusionDetection(bool enable)
{
    if (rectangleSelector) {
        rectangleSelector->EnableOcclusionDetection(enable);
    }
} 