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
    rectangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    rectangleMapper->SetInputData(rectanglePolyData);
    rectangleActor = vtkSmartPointer<vtkActor>::New();
    rectangleActor->SetMapper(rectangleMapper);
    rectangleActor->GetProperty()->SetColor(1.0, 1.0, 0.0);
    rectangleActor->GetProperty()->SetLineWidth(2.0);
    
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
        renderer->AddActor(rectangleActor);
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
    if (renderer) {
        // 只清除选中的点，不清除选择框（选择框应该已经自动消失了）
        renderer->RemoveActor(selectionActor);
        selectedPointIds.clear();
        renderer->GetRenderWindow()->Render();
    }
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
    int x1 = std::max(0, std::min(startX, currentX));
    int x2 = std::min(size[0], std::max(startX, currentX));
    int y1 = std::max(0, std::min(startY, currentY));
    int y2 = std::min(size[1], std::max(startY, currentY));
    vtkCamera* camera = renderer->GetActiveCamera();
    double* bounds = renderer->ComputeVisiblePropBounds();
    double worldPoint1[4], worldPoint2[4], worldPoint3[4], worldPoint4[4];
    renderer->SetDisplayPoint(x1, y1, 0);
    renderer->DisplayToWorld();
    renderer->GetWorldPoint(worldPoint1);
    renderer->SetDisplayPoint(x2, y1, 0);
    renderer->DisplayToWorld();
    renderer->GetWorldPoint(worldPoint2);
    renderer->SetDisplayPoint(x2, y2, 0);
    renderer->DisplayToWorld();
    renderer->GetWorldPoint(worldPoint3);
    renderer->SetDisplayPoint(x1, y2, 0);
    renderer->DisplayToWorld();
    renderer->GetWorldPoint(worldPoint4);
    // 只更新点坐标
    rectanglePoints->SetPoint(0, worldPoint1[0], worldPoint1[1], worldPoint1[2]);
    rectanglePoints->SetPoint(1, worldPoint2[0], worldPoint2[1], worldPoint2[2]);
    rectanglePoints->SetPoint(2, worldPoint3[0], worldPoint3[1], worldPoint3[2]);
    rectanglePoints->SetPoint(3, worldPoint4[0], worldPoint4[1], worldPoint4[2]);
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
    if (selectedPointIds.empty() || !renderer) return;
    
    // 创建选中点的数据
    vtkSmartPointer<vtkPoints> selectedPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");
    
    vtkPoints* originalPoints = originalPointData->GetPoints();
    
    for (vtkIdType id : selectedPointIds) {
        double point[3];
        originalPoints->GetPoint(id, point);
        selectedPoints->InsertNextPoint(point);
        
        // 设置红色高亮
        unsigned char color[3] = {255, 0, 0}; // 红色
        colors->InsertNextTypedTuple(color);
    }
    
    // 创建选中点的多边形数据
    selectionPolyData->SetPoints(selectedPoints);
    selectionPolyData->GetPointData()->SetScalars(colors);
    
    // 创建球体字形来显示点
    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetRadius(0.1); // 点的大小
    sphere->SetPhiResolution(8);
    sphere->SetThetaResolution(8);
    
    vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
    glyph->SetInputData(selectionPolyData);
    glyph->SetSourceConnection(sphere->GetOutputPort());
    glyph->SetColorModeToColorByScalar();
    glyph->SetScaleModeToDataScalingOff();
    
    // 更新映射器
    selectionMapper->SetInputConnection(glyph->GetOutputPort());
    
    // 添加到渲染器
    renderer->AddActor(selectionActor);
    renderer->GetRenderWindow()->Render();
} 