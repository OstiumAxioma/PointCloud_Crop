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

Selector::Selector()
    : rectangleSelectionEnabled(false)
    , isSelecting(false)
    , startX(0)
    , startY(0)
    , currentX(0)
    , currentY(0)
    , selectionShape(SelectionShape::Rectangle)
    , occlusionDetectionEnabled(true)
    , isDrawingPolygon(false)
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

Selector::~Selector()
{
}

void Selector::SetRenderer(vtkRenderer* ren)
{
    renderer = ren;
    if (renderer) {
        renderer->AddActor2D(rectangleActor);
        rectangleActor->SetVisibility(0); // 初始隐藏
    }
}

void Selector::SetPointCloudData(vtkPolyData* pointData)
{
    originalPointData = pointData;
}

void Selector::EnableRectangleSelection(bool enable)
{
    rectangleSelectionEnabled = enable;
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

void Selector::ClearCurrentSelection()
{
    // 清除当前选择状态，包括正在绘制的选择框
    switch (selectionShape) {
        case SelectionShape::Rectangle:
            ClearSelectionRectangle();
            break;
        case SelectionShape::Circle:
            ClearSelectionCircle();
            break;
        case SelectionShape::Polygon:
            ClearSelectionPolygon();
            break;
    }
    
    // 重置选择状态
    isSelecting = false;
    isDrawingPolygon = false;
    
    qDebug() << "已清除当前选择状态";
}

void Selector::OnLeftButtonDown()
{
    if (!rectangleSelectionEnabled) {
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
        return;
    }
    
    int x, y;
    this->GetInteractor()->GetEventPosition(x, y);
    
    if (selectionShape == SelectionShape::Polygon) {
        // 多边形模式下的左键处理
        if (!isDrawingPolygon) {
            // 开始绘制多边形
            isDrawingPolygon = true;
            polygonVertices.clear();
            //等添加第一个顶点后再显示rectangleActor
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
        // 矩形和圆形模式的原有逻辑
        isSelecting = true;
        startX = x;
        startY = y;
        currentX = startX;
        currentY = startY;
        
        // 显示选择框
        rectangleActor->SetVisibility(1);
        DrawSelectionShape();
        
        if (cursorCallback) {
            cursorCallback(Qt::CrossCursor);
        }
    }
}

void Selector::OnLeftButtonUp()
{
    if (!rectangleSelectionEnabled) {
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
        return;
    }
    
    if (selectionShape == SelectionShape::Polygon) {
        // 多边形模式下左键抬起不执行选择，等待右键完成
        return;
    }
    
    if (!isSelecting) {
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

void Selector::OnRightButtonDown()
{
    if (!rectangleSelectionEnabled) {
        vtkInteractorStyleTrackballCamera::OnRightButtonDown();
        return;
    }
    
    if (selectionShape == SelectionShape::Polygon && isDrawingPolygon) {
        // 完成多边形选择
        CompletePolygonSelection();
    } else {
        vtkInteractorStyleTrackballCamera::OnRightButtonDown();
    }
}

void Selector::OnKeyPress()
{
    if (!rectangleSelectionEnabled) {
        vtkInteractorStyleTrackballCamera::OnKeyPress();
        return;
    }
    
    // 获取按键信息
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    
    if (selectionShape == SelectionShape::Polygon && isDrawingPolygon) {
        if (key == "BackSpace") {
            // 撤销最后一个顶点
            UndoLastVertex();
            return;
        }
    }
    
    // 调用父类方法处理其他按键
    vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void Selector::OnMouseMove()
{
    if (!rectangleSelectionEnabled || (!isSelecting && !isDrawingPolygon)) {
        vtkInteractorStyleTrackballCamera::OnMouseMove();
        return;
    }
    
    if (selectionShape == SelectionShape::Polygon && isDrawingPolygon) {
        // 多边形模式下的鼠标移动处理
        this->GetInteractor()->GetEventPosition(currentX, currentY);
        DrawSelectionPolygon();
    } else if (isSelecting) {
        this->GetInteractor()->GetEventPosition(currentX, currentY);
        DrawSelectionShape();
    }
}

void Selector::DrawSelectionRectangle()
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
    
    // 重置点和线为矩形结构
    rectanglePoints->SetNumberOfPoints(4);
    
    // 更新矩形顶点（使用屏幕坐标）
    rectanglePoints->SetPoint(0, x2, vtkY1, 0);
    rectanglePoints->SetPoint(1, x1, vtkY1, 0);
    rectanglePoints->SetPoint(2, x1, vtkY2, 0);
    rectanglePoints->SetPoint(3, x2, vtkY2, 0);
    
    // 更新线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < 4; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i + 1) % 4);
        lines->InsertNextCell(line);
    }
    rectanglePolyData->SetLines(lines);
    
    rectanglePoints->Modified();
    rectanglePolyData->Modified();
    renderer->GetRenderWindow()->Render();
}

void Selector::ClearSelectionRectangle()
{
    if (renderer) {
        rectangleActor->SetVisibility(0);
        renderer->GetRenderWindow()->Render();
    }
}

void Selector::PerformPointSelection()
{
    if (!renderer || !originalPointData) return;
    
    // 使用新的遮挡感知选择算法
    PerformOcclusionAwareSelection();
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

void Selector::PerformOcclusionAwareSelection()
{
    if (!renderer || !originalPointData) return;
    
    // 获取相机和视口信息
    vtkCamera* camera = renderer->GetActiveCamera();
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
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
        
        // 使用通用的区域判断方法
        if (IsPointInSelectionArea(screenPoint[0], screenPoint[1])) {
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

void Selector::DrawSelectionShape()
{
    switch (selectionShape) {
        case SelectionShape::Rectangle:
            DrawSelectionRectangle();
            break;
        case SelectionShape::Circle:
            DrawSelectionCircle();
            break;
        case SelectionShape::Polygon:
            DrawSelectionPolygon();
            break;
    }
}

void Selector::ClearSelectionShape()
{
    switch (selectionShape) {
        case SelectionShape::Rectangle:
            ClearSelectionRectangle();
            break;
        case SelectionShape::Circle:
            ClearSelectionCircle();
            break;
        case SelectionShape::Polygon:
            ClearSelectionPolygon();
            break;
    }
}

void Selector::DrawSelectionCircle()
{
    if (!renderer) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 计算圆心和半径
    double centerX = (startX + currentX) / 2.0;
    double centerY = (startY + currentY) / 2.0;
    double radius = sqrt(pow(currentX - startX, 2) + pow(currentY - startY, 2)) / 2.0;
    
    // 确保圆心在屏幕范围内
    centerX = std::max(radius, std::min(size[0] - radius, centerX));
    centerY = std::max(radius, std::min(size[1] - radius, centerY));
    
    // 生成圆形点集（使用足够多的点来近似圆形）
    const int numPoints = 64;
    rectanglePoints->SetNumberOfPoints(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * M_PI * i / numPoints;
        double x = centerX + radius * cos(angle);
        double y = centerY + radius * sin(angle);
        rectanglePoints->SetPoint(i, x, y, 0);
    }
    
    // 更新线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < numPoints; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i + 1) % numPoints);
        lines->InsertNextCell(line);
    }
    rectanglePolyData->SetLines(lines);
    
    rectanglePoints->Modified();
    rectanglePolyData->Modified();
    renderer->GetRenderWindow()->Render();
}

void Selector::ClearSelectionCircle()
{
    if (renderer) {
        rectangleActor->SetVisibility(0);
        renderer->GetRenderWindow()->Render();
    }
}

bool Selector::IsPointInSelectionArea(double screenX, double screenY)
{
    switch (selectionShape) {
        case SelectionShape::Rectangle: {
            // 矩形选择区域
            double x1 = std::min(startX, currentX);
            double x2 = std::max(startX, currentX);
            double y1 = std::min(startY, currentY);
            double y2 = std::max(startY, currentY);
            
            return (screenX >= x1 && screenX <= x2 && screenY >= y1 && screenY <= y2);
        }
        case SelectionShape::Circle: {
            // 圆形选择区域
            double centerX = (startX + currentX) / 2.0;
            double centerY = (startY + currentY) / 2.0;
            double radius = sqrt(pow(currentX - startX, 2) + pow(currentY - startY, 2)) / 2.0;
            
            double distance = sqrt(pow(screenX - centerX, 2) + pow(screenY - centerY, 2));
            return distance <= radius;
        }
        case SelectionShape::Polygon: {
            // 多边形选择区域
            if (polygonVertices.size() < 3) return false;
            
            std::vector<std::pair<double, double>> polygon;
            for (const auto& vertex : polygonVertices) {
                polygon.emplace_back(vertex.first, vertex.second);
            }
            
            return IsPointInPolygon(screenX, screenY, polygon);
        }
        default:
            return false;
    }
}

void Selector::AddPolygonVertex(int x, int y)
{
    polygonVertices.emplace_back(x, y);
    
    // 添加第一个顶点时才显示选择框
    if (polygonVertices.size() == 1) {
        rectangleActor->SetVisibility(1);
    }
    
    DrawSelectionPolygon(false); // 点击时不添加临时顶点
    qDebug() << "添加多边形顶点:" << x << "," << y << "，当前顶点数:" << polygonVertices.size();
}

void Selector::UndoLastVertex()
{
    if (polygonVertices.empty()) {
        qDebug() << "没有顶点可以撤销";
        return;
    }
    
    // 删除最后一个顶点
    auto lastVertex = polygonVertices.back();
    polygonVertices.pop_back();
    
    qDebug() << "撤销顶点:" << lastVertex.first << "," << lastVertex.second << "，剩余顶点数:" << polygonVertices.size();
    
    if (polygonVertices.empty()) {
        // 如果所有顶点都被删除，取消当前绘制
        isDrawingPolygon = false;
        rectangleActor->SetVisibility(0);
        renderer->GetRenderWindow()->Render();
        
        if (cursorCallback) {
            cursorCallback(Qt::ArrowCursor);
        }
        
        qDebug() << "所有顶点已撤销，取消多边形绘制";
    } else {
        // 重新绘制多边形
        DrawSelectionPolygon(false); // 撤销时不添加临时顶点
    }
}

void Selector::CompletePolygonSelection()
{
    if (polygonVertices.size() < 3) {
        qDebug() << "多边形顶点数不足3个，无法完成选择";
        return;
    }
    
    isDrawingPolygon = false;
    
    // 执行点云选择
    PerformPointSelection();
    
    // 隐藏选择框
    rectangleActor->SetVisibility(0);
    renderer->GetRenderWindow()->Render();
    
    // 清空顶点列表
    polygonVertices.clear();
    
    if (cursorCallback) {
        cursorCallback(Qt::ArrowCursor);
    }
    
    qDebug() << "多边形选择完成";
}

void Selector::DrawSelectionPolygon(bool addTemporaryVertex)
{
    if (!renderer || polygonVertices.empty()) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 基于参数决定是否添加临时顶点
    std::vector<std::pair<int, int>> drawVertices = polygonVertices;
    if (addTemporaryVertex && isDrawingPolygon && polygonVertices.size() >= 1) {
        // 只有在鼠标移动时才添加当前鼠标位置作为临时顶点
        drawVertices.emplace_back(currentX, currentY);
    }
    
    if (drawVertices.size() < 1) return;
    
    // 设置顶点数量
    rectanglePoints->SetNumberOfPoints(drawVertices.size());
    
    // 更新顶点坐标
    for (size_t i = 0; i < drawVertices.size(); ++i) {
        int x = std::max(0, std::min(size[0], drawVertices[i].first));
        int y = std::max(0, std::min(size[1], drawVertices[i].second));
        rectanglePoints->SetPoint(i, x, y, 0);
    }
    
    // 创建线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    
    // 只有在至少有两个顶点时才绘制线条
    if (drawVertices.size() >= 2) {
        // 连接所有相邻顶点
        for (size_t i = 0; i < drawVertices.size() - 1; ++i) {
            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, i);
            line->GetPointIds()->SetId(1, i + 1);
            lines->InsertNextCell(line);
        }
        
        // 如果不是在绘制状态（即已完成），连接最后一个点到第一个点
        if (!isDrawingPolygon && drawVertices.size() > 2) {
            vtkSmartPointer<vtkLine> closingLine = vtkSmartPointer<vtkLine>::New();
            closingLine->GetPointIds()->SetId(0, drawVertices.size() - 1);
            closingLine->GetPointIds()->SetId(1, 0);
            lines->InsertNextCell(closingLine);
        }
    }
    
    rectanglePolyData->SetLines(lines);
    rectanglePoints->Modified();
    rectanglePolyData->Modified();
    renderer->GetRenderWindow()->Render();
}

void Selector::ClearSelectionPolygon()
{
    if (renderer) {
        rectangleActor->SetVisibility(0);
        renderer->GetRenderWindow()->Render();
    }
    polygonVertices.clear();
    isDrawingPolygon = false;
}

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