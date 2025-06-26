#include "selector.h"
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
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

vtkStandardNewMacro(Selector);

// Shape类的containsPoint实现
bool Shape::containsPoint(double x, double y) const {
    switch (type) {
        case SelectionShape::Rectangle: {
            double x1 = std::min(rect.x1, rect.x2);
            double x2 = std::max(rect.x1, rect.x2);
            double y1 = std::min(rect.y1, rect.y2);
            double y2 = std::max(rect.y1, rect.y2);
            return (x >= x1 && x <= x2 && y >= y1 && y <= y2);
        }
        case SelectionShape::Circle: {
            double distance = sqrt(pow(x - circle.centerX, 2) + pow(y - circle.centerY, 2));
            return distance <= circle.radius;
        }
        case SelectionShape::Polygon: {
            if (polygon.vertices.size() < 3) return false;
            
            // 使用射线法判断点是否在多边形内
            bool inside = false;
            size_t j = polygon.vertices.size() - 1;
            
            for (size_t i = 0; i < polygon.vertices.size(); ++i) {
                double xi = polygon.vertices[i].first;
                double yi = polygon.vertices[i].second;
                double xj = polygon.vertices[j].first;
                double yj = polygon.vertices[j].second;
                
                if (((yi > y) != (yj > y)) && 
                    (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
                    inside = !inside;
                }
                j = i;
            }
            return inside;
        }
        default:
            return false;
    }
}

Selector::Selector()
    : drawingModeEnabled(false)
    , isDrawing(false)
    , startX(0)
    , startY(0)
    , currentX(0)
    , currentY(0)
    , currentDrawingShape(SelectionShape::Rectangle)
    , occlusionDetectionEnabled(true)
    , isDrawingPolygon(false)
    , viewLocked(false)
{
    // 初始化当前形状显示
    currentShapePoints = vtkSmartPointer<vtkPoints>::New();
    currentShapePolyData = vtkSmartPointer<vtkPolyData>::New();
    currentShapePolyData->SetPoints(currentShapePoints);
    currentShapeMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    currentShapeMapper->SetInputData(currentShapePolyData);
    currentShapeActor = vtkSmartPointer<vtkActor2D>::New();
    currentShapeActor->SetMapper(currentShapeMapper);
    currentShapeActor->GetProperty()->SetColor(1.0, 1.0, 0.0); // 黄色，表示正在绘制
    currentShapeActor->GetProperty()->SetLineWidth(2.0);
    
    // 初始化画布显示
    canvasPoints = vtkSmartPointer<vtkPoints>::New();
    canvasPolyData = vtkSmartPointer<vtkPolyData>::New();
    canvasPolyData->SetPoints(canvasPoints);
    canvasMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    canvasMapper->SetInputData(canvasPolyData);
    canvasActor = vtkSmartPointer<vtkActor2D>::New();
    canvasActor->SetMapper(canvasMapper);
    canvasActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色，表示已确定的形状
    canvasActor->GetProperty()->SetLineWidth(2.0);
    
    // 设置2D坐标系统为屏幕坐标
    vtkSmartPointer<vtkCoordinate> coord = vtkSmartPointer<vtkCoordinate>::New();
    coord->SetCoordinateSystem(0); // VTK_DISPLAY = 0，屏幕坐标
    currentShapeMapper->SetTransformCoordinate(coord);
    canvasMapper->SetTransformCoordinate(coord);
}

Selector::~Selector()
{
}

void Selector::SetRenderer(vtkRenderer* ren)
{
    renderer = ren;
    if (renderer) {
        renderer->AddActor2D(currentShapeActor);
        renderer->AddActor2D(canvasActor);
        currentShapeActor->SetVisibility(0); // 初始隐藏
        canvasActor->SetVisibility(0); // 初始隐藏
    }
}

void Selector::SetPointCloudData(vtkPolyData* pointData)
{
    originalPointData = pointData;
}

void Selector::EnableDrawingMode(bool enable)
{
    drawingModeEnabled = enable;
    if (!enable) {
        ClearCurrentDrawing();
    }
}

void Selector::ClearCanvas()
{
    canvasShapes.clear();
    canvasActor->SetVisibility(0);
    if (renderer) {
    renderer->GetRenderWindow()->Render();
    }
    qDebug() << "画布已清空";
}

void Selector::ClearCurrentDrawing()
{
    // 清除当前正在绘制的形状
    isDrawing = false;
    isDrawingPolygon = false;
    currentPolygonVertices.clear();
    currentShapeActor->SetVisibility(0);
    
    if (renderer) {
        renderer->GetRenderWindow()->Render();
    }
    
    qDebug() << "已清除当前绘制状态";
}

void Selector::OnLeftButtonDown()
{
    if (!drawingModeEnabled) {
        if (!viewLocked) {
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
        }
        return;
    }
    
    int x, y;
    this->GetInteractor()->GetEventPosition(x, y);
    
    if (currentDrawingShape == SelectionShape::Polygon) {
        // 多边形模式下的左键处理
        if (!isDrawingPolygon) {
            // 开始绘制多边形
            isDrawingPolygon = true;
            currentPolygonVertices.clear();
        }
        
        // 更新当前鼠标位置
        currentX = x;
        currentY = y;
        
        // 添加顶点
        AddPolygonVertex(x, y);
        
        if (cursorCallback) {
            cursorCallback(Qt::CrossCursor);
        }
    } else {
        // 矩形和圆形模式的处理
        isDrawing = true;
        startX = x;
        startY = y;
        currentX = startX;
        currentY = startY;
        
        // 显示当前形状
        currentShapeActor->SetVisibility(1);
        DrawCurrentShape();
        
        if (cursorCallback) {
            cursorCallback(Qt::CrossCursor);
        }
    }
}

void Selector::OnLeftButtonUp()
{
    if (!drawingModeEnabled) {
        if (!viewLocked) {
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
        }
        return;
    }
    
    if (currentDrawingShape == SelectionShape::Polygon) {
        // 多边形模式下左键抬起不执行任何操作，等待右键完成
        return;
    }
    
            if (!isDrawing) {
            if (!viewLocked) {
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
            }
        return;
    }
    
    isDrawing = false;
    
    // 将当前形状添加到画布
    Shape newShape(currentDrawingShape);
    switch (currentDrawingShape) {
        case SelectionShape::Rectangle: {
            newShape.rect.x1 = startX;
            newShape.rect.y1 = startY;
            newShape.rect.x2 = currentX;
            newShape.rect.y2 = currentY;
            break;
        }
        case SelectionShape::Circle: {
            newShape.circle.centerX = (startX + currentX) / 2.0;
            newShape.circle.centerY = (startY + currentY) / 2.0;
            newShape.circle.radius = sqrt(pow(currentX - startX, 2) + pow(currentY - startY, 2)) / 2.0;
            break;
        }
        default:
            break;
    }
    
    canvasShapes.push_back(newShape);
    qDebug() << "形状已添加到画布，当前画布形状数：" << canvasShapes.size();
    
    // 隐藏当前形状显示，显示画布
    currentShapeActor->SetVisibility(0);
    DrawCanvasShapes();
    
    if (cursorCallback) {
        cursorCallback(Qt::ArrowCursor);
    }
}

void Selector::OnRightButtonDown()
{
    if (!drawingModeEnabled) {
        if (!viewLocked) {
        vtkInteractorStyleTrackballCamera::OnRightButtonDown();
        }
        return;
    }
    
    if (currentDrawingShape == SelectionShape::Polygon && isDrawingPolygon) {
        // 完成多边形绘制
        CompleteCurrentPolygon();
    } else {
        if (!viewLocked) {
        vtkInteractorStyleTrackballCamera::OnRightButtonDown();
        }
    }
}

void Selector::OnKeyPress()
{
    if (!drawingModeEnabled) {
        if (!viewLocked) {
        vtkInteractorStyleTrackballCamera::OnKeyPress();
        }
        return;
    }
    
    // 获取按键信息
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    
    if (currentDrawingShape == SelectionShape::Polygon && isDrawingPolygon) {
        if (key == "BackSpace") {
            // 撤销最后一个顶点
            UndoLastVertex();
            return;
        }
    }
    
    // 调用父类方法处理其他按键
    if (!viewLocked) {
    vtkInteractorStyleTrackballCamera::OnKeyPress();
    }
}

void Selector::OnMouseMove()
{
    if (!drawingModeEnabled || (!isDrawing && !isDrawingPolygon)) {
        if (!viewLocked) {
        vtkInteractorStyleTrackballCamera::OnMouseMove();
        }
        return;
    }
    
        this->GetInteractor()->GetEventPosition(currentX, currentY);
    DrawCurrentShape();
}

void Selector::DrawCurrentShape()
{
    switch (currentDrawingShape) {
        case SelectionShape::Rectangle:
            DrawRectangle(std::min(startX, currentX), std::min(startY, currentY), 
                         std::max(startX, currentX), std::max(startY, currentY));
            break;
        case SelectionShape::Circle: {
            double centerX = (startX + currentX) / 2.0;
            double centerY = (startY + currentY) / 2.0;
            double radius = sqrt(pow(currentX - startX, 2) + pow(currentY - startY, 2)) / 2.0;
            DrawCircle(centerX, centerY, radius);
            break;
        }
        case SelectionShape::Polygon: {
            std::vector<std::pair<double, double>> vertices;
            for (const auto& v : currentPolygonVertices) {
                vertices.emplace_back(v.first, v.second);
            }
            if (isDrawingPolygon) {
                vertices.emplace_back(currentX, currentY); // 添加临时顶点
            }
            DrawPolygon(vertices, false);
            break;
        }
    }
}

void Selector::DrawRectangle(double x1, double y1, double x2, double y2)
{
    if (!renderer) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 确保坐标在屏幕范围内
    x1 = std::max(0.0, std::min((double)size[0], x1));
    x2 = std::max(0.0, std::min((double)size[0], x2));
    y1 = std::max(0.0, std::min((double)size[1], y1));
    y2 = std::max(0.0, std::min((double)size[1], y2));
    
    // 重置点和线为矩形结构
    currentShapePoints->SetNumberOfPoints(4);
    
    // 更新矩形顶点（使用屏幕坐标）
    currentShapePoints->SetPoint(0, x2, y1, 0);
    currentShapePoints->SetPoint(1, x1, y1, 0);
    currentShapePoints->SetPoint(2, x1, y2, 0);
    currentShapePoints->SetPoint(3, x2, y2, 0);
    
    // 更新线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < 4; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i + 1) % 4);
        lines->InsertNextCell(line);
    }
    currentShapePolyData->SetLines(lines);
    
    currentShapePoints->Modified();
    currentShapePolyData->Modified();
    renderer->GetRenderWindow()->Render();
}

void Selector::DrawCircle(double centerX, double centerY, double radius)
{
    if (!renderer) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 确保圆心在屏幕范围内
    centerX = std::max(radius, std::min(size[0] - radius, centerX));
    centerY = std::max(radius, std::min(size[1] - radius, centerY));
    
    // 生成圆形点集（使用足够多的点来近似圆形）
    const int numPoints = 64;
    currentShapePoints->SetNumberOfPoints(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * M_PI * i / numPoints;
        double x = centerX + radius * cos(angle);
        double y = centerY + radius * sin(angle);
        currentShapePoints->SetPoint(i, x, y, 0);
    }
    
    // 更新线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < numPoints; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i + 1) % numPoints);
        lines->InsertNextCell(line);
    }
    currentShapePolyData->SetLines(lines);
    
    currentShapePoints->Modified();
    currentShapePolyData->Modified();
        renderer->GetRenderWindow()->Render();
    }

void Selector::DrawPolygon(const std::vector<std::pair<double, double>>& vertices, bool addTemporaryVertex)
{
    if (!renderer || vertices.empty()) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 设置顶点数量
    currentShapePoints->SetNumberOfPoints(vertices.size());
    
    // 更新顶点坐标
    for (size_t i = 0; i < vertices.size(); ++i) {
        double x = std::max(0.0, std::min((double)size[0], vertices[i].first));
        double y = std::max(0.0, std::min((double)size[1], vertices[i].second));
        currentShapePoints->SetPoint(i, x, y, 0);
    }
    
    // 创建线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    
    // 只有在至少有两个顶点时才绘制线条
    if (vertices.size() >= 2) {
        // 连接所有相邻顶点
        for (size_t i = 0; i < vertices.size() - 1; ++i) {
            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, i);
            line->GetPointIds()->SetId(1, i + 1);
            lines->InsertNextCell(line);
        }
        
        // 如果不是在绘制状态（即已完成），连接最后一个点到第一个点
        if (!isDrawingPolygon && vertices.size() > 2) {
            vtkSmartPointer<vtkLine> closingLine = vtkSmartPointer<vtkLine>::New();
            closingLine->GetPointIds()->SetId(0, vertices.size() - 1);
            closingLine->GetPointIds()->SetId(1, 0);
            lines->InsertNextCell(closingLine);
        }
    }
    
    currentShapePolyData->SetLines(lines);
    currentShapePoints->Modified();
    currentShapePolyData->Modified();
    renderer->GetRenderWindow()->Render();
}

void Selector::AddPolygonVertex(int x, int y)
{
    currentPolygonVertices.emplace_back(x, y);
    
    // 添加第一个顶点时才显示选择框
    if (currentPolygonVertices.size() == 1) {
        currentShapeActor->SetVisibility(1);
    }
    
    DrawCurrentShape(); // 使用新的绘制方法
    qDebug() << "添加多边形顶点:" << x << "," << y << "，当前顶点数:" << currentPolygonVertices.size();
}

void Selector::UndoLastVertex()
{
    if (currentPolygonVertices.empty()) {
        qDebug() << "没有顶点可以撤销";
        return;
    }
    
    // 删除最后一个顶点
    auto lastVertex = currentPolygonVertices.back();
    currentPolygonVertices.pop_back();
    
    qDebug() << "撤销顶点:" << lastVertex.first << "," << lastVertex.second << "，剩余顶点数:" << currentPolygonVertices.size();
    
    if (currentPolygonVertices.empty()) {
        // 如果所有顶点都被删除，取消当前绘制
        isDrawingPolygon = false;
        currentShapeActor->SetVisibility(0);
        renderer->GetRenderWindow()->Render();
        
        if (cursorCallback) {
            cursorCallback(Qt::ArrowCursor);
        }
        
        qDebug() << "所有顶点已撤销，取消多边形绘制";
    } else {
        // 重新绘制多边形
        DrawCurrentShape();
    }
}

void Selector::CompleteCurrentPolygon()
{
    if (currentPolygonVertices.size() < 3) {
        qDebug() << "多边形顶点数不足3个，无法完成绘制";
        return;
    }
    
    isDrawingPolygon = false;
    
    // 将多边形添加到画布
    Shape newShape(SelectionShape::Polygon);
    for (const auto& vertex : currentPolygonVertices) {
        newShape.polygon.vertices.emplace_back(vertex.first, vertex.second);
    }
    
    canvasShapes.push_back(newShape);
    qDebug() << "多边形已添加到画布，当前画布形状数：" << canvasShapes.size();
    
    // 隐藏当前形状显示，显示画布
    currentShapeActor->SetVisibility(0);
    DrawCanvasShapes();
    
    // 清空顶点列表
    currentPolygonVertices.clear();
    
    if (cursorCallback) {
        cursorCallback(Qt::ArrowCursor);
    }
    
    qDebug() << "多边形绘制完成";
}

void Selector::DrawCanvasShapes()
{
    if (!renderer || canvasShapes.empty()) {
        canvasActor->SetVisibility(0);
        renderer->GetRenderWindow()->Render();
        return;
    }
    
    // 计算所有形状所需的总点数
    size_t totalPoints = 0;
    for (const auto& shape : canvasShapes) {
        switch (shape.type) {
            case SelectionShape::Rectangle:
                totalPoints += 4;
                break;
            case SelectionShape::Circle:
                totalPoints += 64;
                break;
            case SelectionShape::Polygon:
                totalPoints += shape.polygon.vertices.size();
                break;
        }
    }
    
    if (totalPoints == 0) {
        canvasActor->SetVisibility(0);
        renderer->GetRenderWindow()->Render();
        return;
    }
    
    // 设置总点数
    canvasPoints->SetNumberOfPoints(totalPoints);
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    
    size_t pointIndex = 0;
    
    // 绘制所有形状
    
    for (size_t shapeIdx = 0; shapeIdx < canvasShapes.size(); ++shapeIdx) {
        const auto& shape = canvasShapes[shapeIdx];
        switch (shape.type) {
            case SelectionShape::Rectangle: {
                // 绘制矩形
                double x1 = std::min(shape.rect.x1, shape.rect.x2);
                double x2 = std::max(shape.rect.x1, shape.rect.x2);
                double y1 = std::min(shape.rect.y1, shape.rect.y2);
                double y2 = std::max(shape.rect.y1, shape.rect.y2);
                
                canvasPoints->SetPoint(pointIndex, x2, y1, 0);
                canvasPoints->SetPoint(pointIndex + 1, x1, y1, 0);
                canvasPoints->SetPoint(pointIndex + 2, x1, y2, 0);
                canvasPoints->SetPoint(pointIndex + 3, x2, y2, 0);
                
                // 连接矩形的边
                for (int i = 0; i < 4; ++i) {
                    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                    line->GetPointIds()->SetId(0, pointIndex + i);
                    line->GetPointIds()->SetId(1, pointIndex + (i + 1) % 4);
                    lines->InsertNextCell(line);
                }
                pointIndex += 4;
                break;
            }
            case SelectionShape::Circle: {
                // 绘制圆形
                const int numPoints = 64;
                for (int i = 0; i < numPoints; ++i) {
                    double angle = 2.0 * M_PI * i / numPoints;
                    double x = shape.circle.centerX + shape.circle.radius * cos(angle);
                    double y = shape.circle.centerY + shape.circle.radius * sin(angle);
                    canvasPoints->SetPoint(pointIndex + i, x, y, 0);
                }
                
                // 连接圆形的边
                for (int i = 0; i < numPoints; ++i) {
                    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                    line->GetPointIds()->SetId(0, pointIndex + i);
                    line->GetPointIds()->SetId(1, pointIndex + (i + 1) % numPoints);
                    lines->InsertNextCell(line);
                }
                pointIndex += numPoints;
                break;
            }
            case SelectionShape::Polygon: {
                // 绘制多边形
                for (size_t i = 0; i < shape.polygon.vertices.size(); ++i) {
                    canvasPoints->SetPoint(pointIndex + i, 
                                         shape.polygon.vertices[i].first, 
                                         shape.polygon.vertices[i].second, 0);
                }
                
                // 连接多边形的边
                if (shape.polygon.vertices.size() >= 2) {
                    for (size_t i = 0; i < shape.polygon.vertices.size() - 1; ++i) {
                        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                        line->GetPointIds()->SetId(0, pointIndex + i);
                        line->GetPointIds()->SetId(1, pointIndex + i + 1);
                        lines->InsertNextCell(line);
                    }
                    
                    // 闭合多边形
                    if (shape.polygon.vertices.size() > 2) {
                        vtkSmartPointer<vtkLine> closingLine = vtkSmartPointer<vtkLine>::New();
                        closingLine->GetPointIds()->SetId(0, pointIndex + shape.polygon.vertices.size() - 1);
                        closingLine->GetPointIds()->SetId(1, pointIndex);
                        lines->InsertNextCell(closingLine);
                    }
                }
                pointIndex += shape.polygon.vertices.size();
                break;
            }
        }
    }
    
    canvasPolyData->SetLines(lines);
    canvasPoints->Modified();
    canvasPolyData->Modified();
    
    // 设置画布颜色为更明显的绿色，并增加线宽
    canvasActor->GetProperty()->SetColor(0.0, 1.0, 0.2); // 明亮的绿色
    canvasActor->GetProperty()->SetLineWidth(3.0); // 增加线宽使其更明显
    canvasActor->SetVisibility(1);
    renderer->GetRenderWindow()->Render();
}

void Selector::ConfirmSelection()
{
    if (!renderer || !originalPointData || canvasShapes.empty()) {
        qDebug() << "无法执行选择：缺少必要的数据或画布为空";
        return;
    }
    
    // 基于画布形状进行点云选择
    PerformCanvasBasedSelection();
    
    // 选取完成后自动清空画布
    ClearCanvas();
}

void Selector::ClearAllSelectedPoints()
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

void Selector::PerformCanvasBasedSelection()
{
    if (!renderer || !originalPointData || canvasShapes.empty()) return;
    
    // 获取相机和视口信息
    vtkCamera* camera = renderer->GetActiveCamera();
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    
    // 获取相机位置
    double cameraPos[3];
    camera->GetPosition(cameraPos);
    
    // 收集选择区域内的候选点
    std::vector<vtkIdType> candidatePoints;
    std::map<vtkIdType, std::pair<double, double>> screenPositions; // pointId -> (screenX, screenY)
    std::map<vtkIdType, double> distancesToCamera; // pointId -> distance to camera
    
    vtkPoints* points = originalPointData->GetPoints();
    
    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
        double point[3];
        points->GetPoint(i, point);
        
        // 将3D点投影到屏幕坐标
        double screenPoint[3];
        renderer->SetWorldPoint(point[0], point[1], point[2], 1.0);
        renderer->WorldToDisplay();
        renderer->GetDisplayPoint(screenPoint);
        
        // 使用新的区域判断方法
        if (IsPointInCanvasShapes(screenPoint[0], screenPoint[1])) {
            candidatePoints.push_back(i);

            if (occlusionDetectionEnabled) {
                screenPositions[i] = std::make_pair(screenPoint[0], screenPoint[1]);
                
                // 计算到相机的距离
                double distance = sqrt(pow(point[0] - cameraPos[0], 2) + 
                                     pow(point[1] - cameraPos[1], 2) + 
                                     pow(point[2] - cameraPos[2], 2));
                distancesToCamera[i] = distance;
            }
        }
    }
    
    qDebug() << "选择区域内找到" << candidatePoints.size() << "个候选点";
    
    std::vector<vtkIdType> visiblePoints;
    if (occlusionDetectionEnabled) {
        // 应用遮挡检测过滤
        visiblePoints = FilterOccludedPoints(candidatePoints, screenPositions, distancesToCamera);
    } else {
        // 直接使用所有候选点
        visiblePoints = candidatePoints;
    }
    
    // 将新选中的点添加到已选中的点列表中
    selectedPointIds.insert(selectedPointIds.end(), visiblePoints.begin(), visiblePoints.end());
    
    // 高亮选中的点
    HighlightSelectedPoints(selectedPointIds);
    
    qDebug() << "遮挡检测后选中了" << visiblePoints.size() << "个点，总共选中" << selectedPointIds.size() << "个点";
}

void Selector::HighlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds)
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

void Selector::PerformOcclusionAwareSelection(const std::vector<vtkIdType>& candidatePoints)
{
    if (!renderer || !originalPointData || candidatePoints.empty()) return;
    
    // 获取相机位置
    vtkCamera* camera = renderer->GetActiveCamera();
    double cameraPos[3];
    camera->GetPosition(cameraPos);
    
    std::map<vtkIdType, std::pair<double, double>> screenPositions; // pointId -> (screenX, screenY)
    std::map<vtkIdType, double> distancesToCamera; // pointId -> distance to camera
    
    vtkPoints* points = originalPointData->GetPoints();
    
    // 为候选点计算屏幕位置和距离
    for (vtkIdType pointId : candidatePoints) {
        double point[3];
        points->GetPoint(pointId, point);
        
        // 将3D点投影到屏幕坐标
        double screenPoint[3];
        renderer->SetWorldPoint(point[0], point[1], point[2], 1.0);
        renderer->WorldToDisplay();
        renderer->GetDisplayPoint(screenPoint);
        
        screenPositions[pointId] = std::make_pair(screenPoint[0], screenPoint[1]);
                
                // 计算到相机的距离
                double distance = sqrt(pow(point[0] - cameraPos[0], 2) + 
                                     pow(point[1] - cameraPos[1], 2) + 
                                     pow(point[2] - cameraPos[2], 2));
        distancesToCamera[pointId] = distance;
    }
    
    std::vector<vtkIdType> visiblePoints;
    if (occlusionDetectionEnabled) {
        // 应用遮挡检测过滤
        visiblePoints = FilterOccludedPoints(candidatePoints, screenPositions, distancesToCamera);
    } else {
        // 直接使用所有候选点
        visiblePoints = candidatePoints;
    }
    
    // 将新选中的点添加到已选中的点列表中
    selectedPointIds.insert(selectedPointIds.end(), visiblePoints.begin(), visiblePoints.end());
    
    // 高亮选中的点
    HighlightSelectedPoints(selectedPointIds);
    
    qDebug() << "遮挡检测后选中了" << visiblePoints.size() << "个点，总共选中" << selectedPointIds.size() << "个点";
}

std::vector<vtkIdType> Selector::FilterOccludedPoints(const std::vector<vtkIdType>& candidatePoints,
                                                    const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                                                    const std::map<vtkIdType, double>& distancesToCamera)
{
    if (candidatePoints.empty()) return {};
    
    // 按距离排序（从近到远）
    std::vector<vtkIdType> sortedPoints = candidatePoints;
    std::sort(sortedPoints.begin(), sortedPoints.end(), 
              [&distancesToCamera](vtkIdType a, vtkIdType b) {
                  return distancesToCamera.at(a) < distancesToCamera.at(b);
              });
    
    // 基于深度缓冲的遮挡检测
    std::vector<vtkIdType> visiblePoints;
    double pixelThreshold = 6.0; // 像素距离阈值
    
    // 创建深度缓冲区（简化为距离映射）
    std::map<std::pair<int, int>, double> depthBuffer; // (pixelX, pixelY) -> minDistance
    
    for (vtkIdType pointId : sortedPoints) {
        auto it = screenPositions.find(pointId);
        if (it == screenPositions.end()) continue;
        
        double screenX = it->second.first;
        double screenY = it->second.second;
        double pointDistance = distancesToCamera.at(pointId);
        
        // 检查周围像素区域
        bool isOccluded = false;
        int pixelX = static_cast<int>(screenX);
        int pixelY = static_cast<int>(screenY);
        
        // 检查周围区域是否已有更近的点
        for (int dx = -static_cast<int>(pixelThreshold); dx <= static_cast<int>(pixelThreshold); ++dx) {
            for (int dy = -static_cast<int>(pixelThreshold); dy <= static_cast<int>(pixelThreshold); ++dy) {
                int checkX = pixelX + dx;
                int checkY = pixelY + dy;
                
                auto bufferIt = depthBuffer.find(std::make_pair(checkX, checkY));
                if (bufferIt != depthBuffer.end()) {
                    double existingDistance = bufferIt->second;
                    // 如果已有更近的点，且距离差异足够大，认为被遮挡
                    if (existingDistance < pointDistance && 
                        (pointDistance - existingDistance) / existingDistance > 0.02) {
                        isOccluded = true;
                        break;
                    }
                }
            }
            if (isOccluded) break;
        }
        
        if (!isOccluded) {
            visiblePoints.push_back(pointId);
            
            // 更新深度缓冲区
            for (int dx = -static_cast<int>(pixelThreshold); dx <= static_cast<int>(pixelThreshold); ++dx) {
                for (int dy = -static_cast<int>(pixelThreshold); dy <= static_cast<int>(pixelThreshold); ++dy) {
                    int updateX = pixelX + dx;
                    int updateY = pixelY + dy;
                    
                    auto bufferIt = depthBuffer.find(std::make_pair(updateX, updateY));
                    if (bufferIt == depthBuffer.end() || bufferIt->second > pointDistance) {
                        depthBuffer[std::make_pair(updateX, updateY)] = pointDistance;
                    }
                }
            }
        }
    }
    
    return visiblePoints;
}

bool Selector::IsPointOccluded(vtkIdType pointId, const std::vector<vtkIdType>& frontPoints, 
                                       const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                                       double occlusionThreshold)
{
    if (frontPoints.empty()) return false;
    
    auto it = screenPositions.find(pointId);
    if (it == screenPositions.end()) return false;
    
    double pointX = it->second.first;
    double pointY = it->second.second;
    
    // 检查是否被前面的点遮挡
    for (vtkIdType frontPointId : frontPoints) {
        auto frontIt = screenPositions.find(frontPointId);
        if (frontIt == screenPositions.end()) continue;
        
        double frontX = frontIt->second.first;
        double frontY = frontIt->second.second;
        
        // 计算屏幕距离
        double distance = CalculateScreenDistance(pointX, pointY, frontX, frontY);
        
        // 如果距离小于阈值，认为被遮挡
        if (distance < occlusionThreshold) {
            return true;
        }
    }
    
    return false;
}

double Selector::CalculateScreenDistance(double x1, double y1, double x2, double y2)
{
    // 计算屏幕距离
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// 该方法已被移除，功能由 IsPointInCanvasShapes 替代

bool Selector::IsPointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon)
{
    if (polygon.size() < 3) return false;
    
    // 使用射线法判断点是否在多边形内
    bool inside = false;
    size_t j = polygon.size() - 1;
    
    for (size_t i = 0; i < polygon.size(); ++i) {
        double xi = polygon[i].first;
        double yi = polygon[i].second;
        double xj = polygon[j].first;
        double yj = polygon[j].second;
        
        // 检查射线是否与边相交
        if (((yi > y) != (yj > y)) && 
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
        j = i;
    }
    
    return inside;
}

bool Selector::IsPointInCanvasShapes(double screenX, double screenY)
{
    for (const auto& shape : canvasShapes) {
        if (shape.containsPoint(screenX, screenY)) {
            return true;
        }
    }
    return false;
}