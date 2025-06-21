#include "rectangleselector.h"
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkFrustumSelector.h>
#include <vtkExtractSelectedFrustum.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkIdTypeArray.h>
#include <vtkExtractSelection.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <QDebug>

vtkStandardNewMacro(RectangleSelector);

RectangleSelector::RectangleSelector()
    : rectangleSelectionEnabled(false)
    , isSelecting(false)
    , startX(0)
    , startY(0)
    , currentX(0)
    , currentY(0)
{
    rectanglePoints = vtkSmartPointer<vtkPoints>::New();
    rectanglePoints->SetNumberOfPoints(4); // 固定4个点
    rectanglePolyData = vtkSmartPointer<vtkPolyData>::New();
    rectanglePolyData->SetPoints(rectanglePoints);
    // 初始化线条
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < 4; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i+1)%4);
        lines->InsertNextCell(line);
    }
    rectanglePolyData->SetLines(lines);
    rectangleMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    rectangleMapper->SetInputData(rectanglePolyData);
    rectangleActor = vtkSmartPointer<vtkActor2D>::New();
    rectangleActor->SetMapper(rectangleMapper);
    rectangleActor->GetProperty()->SetColor(1.0, 1.0, 0.0);
    rectangleActor->GetProperty()->SetLineWidth(2.0);
    
    // 设置2D坐标系统为屏幕坐标
    vtkSmartPointer<vtkCoordinate> coord = vtkSmartPointer<vtkCoordinate>::New();
    coord->SetCoordinateSystem(0); // VTK_DISPLAY = 0，屏幕坐标
    rectangleMapper->SetTransformCoordinate(coord);
    
    // 初始化选择结果相关对象
    selectionPolyData = vtkSmartPointer<vtkPolyData>::New();
    selectionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    selectionActor = vtkSmartPointer<vtkActor>::New();
    selectionActor->SetMapper(selectionMapper);
    selectionActor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    selectionActor->GetProperty()->SetPointSize(3.0);
}

RectangleSelector::~RectangleSelector()
{
}

void RectangleSelector::SetRenderer(vtkRenderer* ren)
{
    renderer = ren;
    if (renderer) {
        renderer->AddActor2D(rectangleActor);
        rectangleActor->SetVisibility(0); // 初始隐藏
    }
}

void RectangleSelector::SetPointCloudData(vtkPolyData* pointData)
{
    originalPointData = pointData;
}

void RectangleSelector::EnableRectangleSelection(bool enable)
{
    rectangleSelectionEnabled = enable;
}

void RectangleSelector::ClearAllSelectedPoints()
{
    if (!renderer || !originalPointData || originalColorBackup.empty()) return;
    
    // 获取原始点云的颜色数据
    vtkUnsignedCharArray* originalColors = vtkUnsignedCharArray::SafeDownCast(
        originalPointData->GetPointData()->GetScalars());
    
    if (!originalColors) return;
    
    // 恢复所有点的原始颜色
    for (vtkIdType i = 0; i < originalColors->GetNumberOfTuples(); ++i) {
        unsigned char* color = originalColors->GetPointer(i * 3);
        color[0] = originalColorBackup[i * 3];
        color[1] = originalColorBackup[i * 3 + 1];
        color[2] = originalColorBackup[i * 3 + 2];
    }
    
    // 标记颜色数据已修改
    originalColors->Modified();
    
    // 清空选中点列表
    selectedPointIds.clear();
    
    // 刷新渲染
    renderer->GetRenderWindow()->Render();
    
    qDebug() << "已清除所有选中点";
}

void RectangleSelector::OnLeftButtonDown()
{
    if (!rectangleSelectionEnabled) {
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
        return;
    }
    
    isSelecting = true;
    this->GetInteractor()->GetEventPosition(startX, startY);
    currentX = startX;
    currentY = startY;
    
    // 显示选择框
    rectangleActor->SetVisibility(1);
    DrawSelectionRectangle();
    
    if (cursorCallback) {
        cursorCallback(Qt::CrossCursor);
    }
}

void RectangleSelector::OnLeftButtonUp()
{
    if (!rectangleSelectionEnabled || !isSelecting) {
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
        return;
    }
    
    isSelecting = false;
    PerformPointSelection();
    
    // 隐藏选择框
    rectangleActor->SetVisibility(0);
    renderer->GetRenderWindow()->Render();
    
    if (cursorCallback) {
        cursorCallback(Qt::CrossCursor);
    }
}

void RectangleSelector::OnMouseMove()
{
    if (!rectangleSelectionEnabled || !isSelecting) {
        vtkInteractorStyleTrackballCamera::OnMouseMove();
        return;
    }
    
    this->GetInteractor()->GetEventPosition(currentX, currentY);
    DrawSelectionRectangle();
}

void RectangleSelector::DrawSelectionRectangle()
{
    if (!renderer) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 确保坐标在屏幕范围内
    int x1 = std::max(0, std::min(startX, currentX));
    int x2 = std::min(size[0], std::max(startX, currentX));
    int y1 = std::max(0, std::min(startY, currentY));
    int y2 = std::min(size[1], std::max(startY, currentY));
    
    // 直接使用Qt的Y坐标，不进行翻转
    int vtkY1 = y1;
    int vtkY2 = y2;
    
    // 更新矩形顶点（使用屏幕坐标）
    rectanglePoints->SetPoint(0, x2, vtkY1, 0);
    rectanglePoints->SetPoint(1, x1, vtkY1, 0);
    rectanglePoints->SetPoint(2, x1, vtkY2, 0);
    rectanglePoints->SetPoint(3, x2, vtkY2, 0);
    
    rectanglePoints->Modified();
    rectanglePolyData->Modified();
    renderer->GetRenderWindow()->Render();
}

void RectangleSelector::ClearSelectionRectangle()
{
    if (renderer) {
        rectangleActor->SetVisibility(0);
        renderer->GetRenderWindow()->Render();
    }
}

void RectangleSelector::PerformPointSelection()
{
    if (!renderer || !originalPointData) return;
    
    // 获取相机和视口信息
    vtkCamera* camera = renderer->GetActiveCamera();
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 计算选择区域
    double x1 = std::min(startX, currentX);
    double x2 = std::max(startX, currentX);
    double y1 = std::min(startY, currentY);
    double y2 = std::max(startY, currentY);
    
    // 转换为标准化坐标
    double nx1 = x1 / size[0];
    double nx2 = x2 / size[0];
    double ny1 = y1 / size[1];
    double ny2 = y2 / size[1];
    
    // 创建视锥体选择器
    vtkSmartPointer<vtkFrustumSelector> selector = vtkSmartPointer<vtkFrustumSelector>::New();
    
    // 设置选择区域
    double frustum[24];
    // 这里需要根据相机参数计算视锥体
    // 简化实现：使用屏幕坐标进行选择
    
    // 使用点选择器进行简化实现
    std::vector<vtkIdType> newSelectedPointIds;
    vtkPoints* points = originalPointData->GetPoints();
    
    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
        double point[3];
        points->GetPoint(i, point);
        
        // 将3D点投影到屏幕坐标
        double screenPoint[3];
        renderer->SetWorldPoint(point[0], point[1], point[2], 1.0);
        renderer->WorldToDisplay();
        renderer->GetDisplayPoint(screenPoint);
        
        // 检查点是否在选择矩形内
        if (screenPoint[0] >= x1 && screenPoint[0] <= x2 &&
            screenPoint[1] >= y1 && screenPoint[1] <= y2) {
            newSelectedPointIds.push_back(i);
        }
    }
    
    // 将新选中的点添加到已选中的点列表中
    selectedPointIds.insert(selectedPointIds.end(), newSelectedPointIds.begin(), newSelectedPointIds.end());
    
    // 高亮选中的点
    HighlightSelectedPoints(selectedPointIds);
    
    qDebug() << "选中了" << newSelectedPointIds.size() << "个点，总共选中" << selectedPointIds.size() << "个点";
}

void RectangleSelector::HighlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds)
{
    if (!renderer || !originalPointData) return;

    // 获取原始点云的颜色数据或标量数据
    vtkUnsignedCharArray* originalColors = vtkUnsignedCharArray::SafeDownCast(
        originalPointData->GetPointData()->GetScalars());

    // 如果没有颜色数据，尝试从标量数据创建颜色
    if (!originalColors) {
        vtkDataArray* scalars = originalPointData->GetPointData()->GetScalars();
        if (scalars) {
            qDebug() << "找到标量数据，正在转换为颜色数据...";

            // 创建颜色数组
            vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            colors->SetNumberOfComponents(3);
            colors->SetName("Colors");
            colors->SetNumberOfTuples(scalars->GetNumberOfTuples());

            // 创建查找表来将标量值转换为颜色
            vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
            lut->SetHueRange(0.667, 0.0); // 蓝色到红色
            lut->SetSaturationRange(1.0, 1.0);
            lut->SetValueRange(1.0, 1.0);
            lut->Build();

            // 获取标量范围
            double range[2];
            scalars->GetRange(range);

            // 为所有点分配颜色
            for (vtkIdType i = 0; i < scalars->GetNumberOfTuples(); ++i) {
                double scalarValue = scalars->GetTuple1(i);
                double normalizedValue = (scalarValue - range[0]) / (range[1] - range[0]);
                double color[3];
                lut->GetColor(normalizedValue, color);
                unsigned char rgb[3];
                rgb[0] = static_cast<unsigned char>(color[0] * 255);
                rgb[1] = static_cast<unsigned char>(color[1] * 255);
                rgb[2] = static_cast<unsigned char>(color[2] * 255);
                colors->SetTypedTuple(i, rgb);
            }

            // 将颜色数据添加到点云
            originalPointData->GetPointData()->SetScalars(colors);
            originalColors = colors;

            qDebug() << "已创建颜色数据，点数:" << colors->GetNumberOfTuples();
        } else {
            qDebug() << "原始点云既没有颜色数据也没有标量数据";
            return;
        }
    }

    // 保存原始颜色（如果还没有保存）
    if (originalColorBackup.empty()) {
        originalColorBackup.resize(originalColors->GetNumberOfTuples() * 3);
        for (vtkIdType i = 0; i < originalColors->GetNumberOfTuples(); ++i) {
            unsigned char* color = originalColors->GetPointer(i * 3);
            originalColorBackup[i * 3] = color[0];
            originalColorBackup[i * 3 + 1] = color[1];
            originalColorBackup[i * 3 + 2] = color[2];
        }
        qDebug() << "已保存原始颜色备份，点数:" << originalColors->GetNumberOfTuples();
    }

    // 先恢复所有点为原色
    for (vtkIdType i = 0; i < originalColors->GetNumberOfTuples(); ++i) {
        unsigned char* color = originalColors->GetPointer(i * 3);
        color[0] = originalColorBackup[i * 3];
        color[1] = originalColorBackup[i * 3 + 1];
        color[2] = originalColorBackup[i * 3 + 2];
    }

    // 将选中的点设置为红色
    for (vtkIdType id : selectedPointIds) {
        if (id < originalColors->GetNumberOfTuples()) {
            unsigned char* color = originalColors->GetPointer(id * 3);
            color[0] = 255; // 红色
            color[1] = 0;   // 绿色
            color[2] = 0;   // 蓝色
        }
    }

    // 标记颜色数据已修改
    originalColors->Modified();

    // 刷新渲染
    renderer->GetRenderWindow()->Render();

    qDebug() << "已高亮" << selectedPointIds.size() << "个点";
} 