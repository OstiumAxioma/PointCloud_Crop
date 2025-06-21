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
    // 初始化选择框相关对象
    rectanglePoints = vtkSmartPointer<vtkPoints>::New();
    rectanglePolyData = vtkSmartPointer<vtkPolyData>::New();
    rectangleActor = vtkSmartPointer<vtkActor>::New();
    rectangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    
    // 初始化选择结果相关对象
    selectionPolyData = vtkSmartPointer<vtkPolyData>::New();
    selectionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    selectionActor = vtkSmartPointer<vtkActor>::New();
    
    // 设置选择框的样式
    rectangleActor->SetMapper(rectangleMapper);
    rectangleActor->GetProperty()->SetColor(1.0, 1.0, 0.0); // 黄色边框
    rectangleActor->GetProperty()->SetLineWidth(2.0);
    rectangleActor->GetProperty()->SetRepresentationToWireframe();
    
    // 设置选中点的样式
    selectionActor->SetMapper(selectionMapper);
    selectionActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色高亮
    selectionActor->GetProperty()->SetPointSize(3.0);
}

RectangleSelector::~RectangleSelector()
{
}

void RectangleSelector::SetRenderer(vtkRenderer* ren)
{
    renderer = ren;
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
        // 如果矩形选择未启用，使用默认的轨迹球相机交互
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
        return;
    }
    
    // 开始矩形选择
    isSelecting = true;
    this->GetInteractor()->GetEventPosition(startX, startY);
    currentX = startX;
    currentY = startY;
    
    // 清除之前的选择
    ClearSelectionRectangle();
    
    // 调用光标回调函数
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
    
    // 结束矩形选择
    isSelecting = false;
    
    // 执行点云选择
    PerformPointSelection();
    
    // 清除选择框
    ClearSelectionRectangle();
    
    // 调用光标回调函数
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
    
    // 更新当前鼠标位置
    this->GetInteractor()->GetEventPosition(currentX, currentY);
    
    // 绘制选择矩形
    DrawSelectionRectangle();
}

void RectangleSelector::DrawSelectionRectangle()
{
    if (!renderer) return;
    
    // 清除之前的选择框
    ClearSelectionRectangle();
    
    // 创建矩形的四个顶点
    rectanglePoints->Reset();
    rectanglePoints->InsertNextPoint(startX, startY, 0);
    rectanglePoints->InsertNextPoint(currentX, startY, 0);
    rectanglePoints->InsertNextPoint(currentX, currentY, 0);
    rectanglePoints->InsertNextPoint(startX, currentY, 0);
    
    // 创建矩形线条
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    
    // 添加四条边
    vtkSmartPointer<vtkLine> line1 = vtkSmartPointer<vtkLine>::New();
    line1->GetPointIds()->SetId(0, 0);
    line1->GetPointIds()->SetId(1, 1);
    
    vtkSmartPointer<vtkLine> line2 = vtkSmartPointer<vtkLine>::New();
    line2->GetPointIds()->SetId(0, 1);
    line2->GetPointIds()->SetId(1, 2);
    
    vtkSmartPointer<vtkLine> line3 = vtkSmartPointer<vtkLine>::New();
    line3->GetPointIds()->SetId(0, 2);
    line3->GetPointIds()->SetId(1, 3);
    
    vtkSmartPointer<vtkLine> line4 = vtkSmartPointer<vtkLine>::New();
    line4->GetPointIds()->SetId(0, 3);
    line4->GetPointIds()->SetId(1, 0);
    
    lines->InsertNextCell(line1);
    lines->InsertNextCell(line2);
    lines->InsertNextCell(line3);
    lines->InsertNextCell(line4);
    
    // 设置多边形数据
    rectanglePolyData->SetPoints(rectanglePoints);
    rectanglePolyData->SetLines(lines);
    
    // 更新映射器
    rectangleMapper->SetInputData(rectanglePolyData);
    
    // 添加到渲染器
    renderer->AddActor(rectangleActor);
    renderer->GetRenderWindow()->Render();
}

void RectangleSelector::ClearSelectionRectangle()
{
    if (renderer) {
        renderer->RemoveActor(rectangleActor);
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