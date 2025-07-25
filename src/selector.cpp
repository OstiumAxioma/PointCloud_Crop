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
#include <vtkVertex.h>
#include <vtkActorCollection.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <QDebug>
#include <QString>
#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>
#include <set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

vtkStandardNewMacro(Selector);

//=============================================================================
// 统一的状态变更命令实现
//=============================================================================

StateChangeCommand::StateChangeCommand(VectorShape* shape, const std::string& beforeState, const std::string& afterState, const std::string& description)
    : shape_(shape), beforeState_(beforeState), afterState_(afterState), description_(description) {
}

void StateChangeCommand::execute() {
    shape_->deserialize(afterState_);
}

void StateChangeCommand::undo() {
    shape_->deserialize(beforeState_);
}

std::string StateChangeCommand::getDescription() const {
    return description_;
}

//=============================================================================
// VectorRectangle 实现
//=============================================================================

VectorRectangle::VectorRectangle(double x1, double y1, double x2, double y2)
    : x1_(x1), y1_(y1), x2_(x2), y2_(y2)
    , dragStartX_(0), dragStartY_(0)
    , originalX1_(0), originalY1_(0), originalX2_(0), originalY2_(0)
{
    normalizeCoordinates();
}

void VectorRectangle::normalizeCoordinates() {
    if (x1_ > x2_) std::swap(x1_, x2_);
    if (y1_ > y2_) std::swap(y1_, y2_);
}

std::vector<std::pair<double, double>> VectorRectangle::getControlPoints() const {
    return {
        {x1_, y1_}, {x2_, y1_}, {x2_, y2_}, {x1_, y2_}, // 四个角
        {(x1_ + x2_) / 2, y1_}, {x2_, (y1_ + y2_) / 2}, 
        {(x1_ + x2_) / 2, y2_}, {x1_, (y1_ + y2_) / 2}  // 四个边中点
    };
}

void VectorRectangle::draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, 
                          vtkPolyData* polyData, vtkPoints* points) {
    // 设置矩形顶点
    points->SetNumberOfPoints(4);
    points->SetPoint(0, x2_, y1_, 0); // 右上
    points->SetPoint(1, x1_, y1_, 0); // 左上
    points->SetPoint(2, x1_, y2_, 0); // 左下
    points->SetPoint(3, x2_, y2_, 0); // 右下
    
    // 设置线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < 4; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i + 1) % 4);
        lines->InsertNextCell(line);
    }
    polyData->SetLines(lines);
    
    // 设置颜色
    if (isSelected_) {
        actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色表示选中
        actor->GetProperty()->SetLineWidth(3.0);
    } else {
        actor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色表示正常
        actor->GetProperty()->SetLineWidth(2.0);
    }
}

bool VectorRectangle::hitTest(double x, double y, double tolerance) const {
    // 检查是否在矩形边界附近
    return (x >= x1_ - tolerance && x <= x2_ + tolerance && 
            y >= y1_ - tolerance && y <= y2_ + tolerance) &&
           (x <= x1_ + tolerance || x >= x2_ - tolerance || 
            y <= y1_ + tolerance || y >= y2_ - tolerance);
}

bool VectorRectangle::containsPoint(double x, double y) const {
    double x1 = std::min(x1_, x2_);
    double x2 = std::max(x1_, x2_);
    double y1 = std::min(y1_, y2_);
    double y2 = std::max(y1_, y2_);
    return (x >= x1 && x <= x2 && y >= y1 && y <= y2);
}

bool VectorRectangle::hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance) const {
    auto controlPts = getControlPoints();
    for (size_t i = 0; i < controlPts.size(); ++i) {
        double dx = x - controlPts[i].first;
        double dy = y - controlPts[i].second;
        if (sqrt(dx * dx + dy * dy) <= tolerance) {
            controlPointIndex = static_cast<int>(i);
            return true;
        }
    }
    return false;
}

void VectorRectangle::startDrag(double x, double y) {
    dragStartX_ = x;
    dragStartY_ = y;
    originalX1_ = x1_;
    originalY1_ = y1_;
    originalX2_ = x2_;
    originalY2_ = y2_;
}

void VectorRectangle::updateDrag(double x, double y, bool shiftPressed) {
    double dx = x - dragStartX_;
    double dy = y - dragStartY_;
    
    x1_ = originalX1_ + dx;
    y1_ = originalY1_ + dy;
    x2_ = originalX2_ + dx;
    y2_ = originalY2_ + dy;
}

void VectorRectangle::endDrag() {
    normalizeCoordinates();
}

void VectorRectangle::moveControlPoint(int index, double x, double y) {
    switch (index) {
        case 0: x1_ = x; y1_ = y; break; // 左上角
        case 1: x2_ = x; y1_ = y; break; // 右上角
        case 2: x2_ = x; y2_ = y; break; // 右下角
        case 3: x1_ = x; y2_ = y; break; // 左下角
        case 4: y1_ = y; break; // 上边中点
        case 5: x2_ = x; break; // 右边中点
        case 6: y2_ = y; break; // 下边中点
        case 7: x1_ = x; break; // 左边中点
    }
    normalizeCoordinates();
}

VectorShape::ShapeData VectorRectangle::toShapeData() const {
    ShapeData data(SelectionShape::Rectangle);
    data.rect.x1 = x1_;
    data.rect.y1 = y1_;
    data.rect.x2 = x2_;
    data.rect.y2 = y2_;
    return data;
}

std::string VectorRectangle::serialize() const {
    return std::to_string(x1_) + "," + std::to_string(y1_) + "," + 
           std::to_string(x2_) + "," + std::to_string(y2_);
}

void VectorRectangle::deserialize(const std::string& data) {
    size_t pos1 = data.find(',');
    size_t pos2 = data.find(',', pos1 + 1);
    size_t pos3 = data.find(',', pos2 + 1);
    
    x1_ = std::stod(data.substr(0, pos1));
    y1_ = std::stod(data.substr(pos1 + 1, pos2 - pos1 - 1));
    x2_ = std::stod(data.substr(pos2 + 1, pos3 - pos2 - 1));
    y2_ = std::stod(data.substr(pos3 + 1));
    
    normalizeCoordinates();
}

//=============================================================================
// VectorCircle 实现
//=============================================================================

VectorCircle::VectorCircle(double centerX, double centerY, double radius)
    : centerX_(centerX), centerY_(centerY), radius_(radius)
    , dragStartX_(0), dragStartY_(0)
    , originalCenterX_(0), originalCenterY_(0), originalRadius_(0)
{
}

std::vector<std::pair<double, double>> VectorCircle::getControlPoints() const {
    return {
        {centerX_ + radius_, centerY_}, // 右
        {centerX_, centerY_ - radius_}, // 上
        {centerX_ - radius_, centerY_}, // 左
        {centerX_, centerY_ + radius_}  // 下
    };
}

void VectorCircle::draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, 
                       vtkPolyData* polyData, vtkPoints* points) {
    const int numPoints = 64;
    points->SetNumberOfPoints(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * M_PI * i / numPoints;
        double x = centerX_ + radius_ * cos(angle);
        double y = centerY_ + radius_ * sin(angle);
        points->SetPoint(i, x, y, 0);
    }
    
    // 设置线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < numPoints; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i + 1) % numPoints);
        lines->InsertNextCell(line);
    }
    polyData->SetLines(lines);
    
    // 设置颜色
    if (isSelected_) {
        actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色表示选中
        actor->GetProperty()->SetLineWidth(3.0);
    } else {
        actor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色表示正常
        actor->GetProperty()->SetLineWidth(2.0);
    }
}

bool VectorCircle::hitTest(double x, double y, double tolerance) const {
    double distance = sqrt(pow(x - centerX_, 2) + pow(y - centerY_, 2));
    return abs(distance - radius_) <= tolerance;
}

bool VectorCircle::containsPoint(double x, double y) const {
    double distance = sqrt(pow(x - centerX_, 2) + pow(y - centerY_, 2));
    return distance <= radius_;
}

bool VectorCircle::hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance) const {
    auto controlPts = getControlPoints();
    for (size_t i = 0; i < controlPts.size(); ++i) {
        double dx = x - controlPts[i].first;
        double dy = y - controlPts[i].second;
        if (sqrt(dx * dx + dy * dy) <= tolerance) {
            controlPointIndex = static_cast<int>(i);
            return true;
        }
    }
    return false;
}

void VectorCircle::startDrag(double x, double y) {
    dragStartX_ = x;
    dragStartY_ = y;
    originalCenterX_ = centerX_;
    originalCenterY_ = centerY_;
    originalRadius_ = radius_;
}

void VectorCircle::updateDrag(double x, double y, bool shiftPressed) {
    double dx = x - dragStartX_;
    double dy = y - dragStartY_;
    
    centerX_ = originalCenterX_ + dx;
    centerY_ = originalCenterY_ + dy;
}

void VectorCircle::endDrag() {
    // 圆形拖拽结束无需特殊处理
}

void VectorCircle::moveControlPoint(int index, double x, double y) {
    // 计算新半径
    double dx = x - centerX_;
    double dy = y - centerY_;
    radius_ = sqrt(dx * dx + dy * dy);
}

VectorShape::ShapeData VectorCircle::toShapeData() const {
    ShapeData data(SelectionShape::Circle);
    data.circle.centerX = centerX_;
    data.circle.centerY = centerY_;
    data.circle.radius = radius_;
    return data;
}

std::string VectorCircle::serialize() const {
    return std::to_string(centerX_) + "," + std::to_string(centerY_) + "," + 
           std::to_string(radius_);
}

void VectorCircle::deserialize(const std::string& data) {
    size_t pos1 = data.find(',');
    size_t pos2 = data.find(',', pos1 + 1);
    
    centerX_ = std::stod(data.substr(0, pos1));
    centerY_ = std::stod(data.substr(pos1 + 1, pos2 - pos1 - 1));
    radius_ = std::stod(data.substr(pos2 + 1));
}

//=============================================================================
// VectorPolygon 实现
//=============================================================================

VectorPolygon::VectorPolygon(const std::vector<std::pair<double, double>>& vertices)
    : vertices_(vertices)
    , dragStartX_(0), dragStartY_(0)
{
}

bool VectorPolygon::isPointInPolygon(double x, double y) const {
    if (vertices_.size() < 3) return false;
    
    bool inside = false;
    size_t j = vertices_.size() - 1;
    
    for (size_t i = 0; i < vertices_.size(); ++i) {
        double xi = vertices_[i].first;
        double yi = vertices_[i].second;
        double xj = vertices_[j].first;
        double yj = vertices_[j].second;
        
        if (((yi > y) != (yj > y)) && 
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
        j = i;
    }
    
    return inside;
}

void VectorPolygon::draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, 
                        vtkPolyData* polyData, vtkPoints* points) {
    if (vertices_.empty()) return;
    
    points->SetNumberOfPoints(vertices_.size());
    
    for (size_t i = 0; i < vertices_.size(); ++i) {
        points->SetPoint(i, vertices_[i].first, vertices_[i].second, 0);
    }
    
    // 设置线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    if (vertices_.size() >= 2) {
        for (size_t i = 0; i < vertices_.size() - 1; ++i) {
            vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetId(0, i);
            line->GetPointIds()->SetId(1, i + 1);
            lines->InsertNextCell(line);
        }
        
        // 闭合多边形
        if (vertices_.size() > 2) {
            vtkSmartPointer<vtkLine> closingLine = vtkSmartPointer<vtkLine>::New();
            closingLine->GetPointIds()->SetId(0, vertices_.size() - 1);
            closingLine->GetPointIds()->SetId(1, 0);
            lines->InsertNextCell(closingLine);
        }
    }
    polyData->SetLines(lines);
    
    // 设置颜色
    if (isSelected_) {
        actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色表示选中
        actor->GetProperty()->SetLineWidth(3.0);
    } else {
        actor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色表示正常
        actor->GetProperty()->SetLineWidth(2.0);
    }
}

bool VectorPolygon::hitTest(double x, double y, double tolerance) const {
    if (vertices_.size() < 2) return false;
    
    // 检查是否在多边形边界附近
    for (size_t i = 0; i < vertices_.size(); ++i) {
        size_t j = (i + 1) % vertices_.size();
        double x1 = vertices_[i].first;
        double y1 = vertices_[i].second;
        double x2 = vertices_[j].first;
        double y2 = vertices_[j].second;
        
        // 计算点到线段的距离
        double A = x - x1;
        double B = y - y1;
        double C = x2 - x1;
        double D = y2 - y1;
        
        double dot = A * C + B * D;
        double lenSq = C * C + D * D;
        double param = (lenSq != 0) ? dot / lenSq : -1;
        
        double xx, yy;
        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }
        
        double dx = x - xx;
        double dy = y - yy;
        double distance = sqrt(dx * dx + dy * dy);
        
        if (distance <= tolerance) {
            return true;
        }
    }
    
    return false;
}

bool VectorPolygon::containsPoint(double x, double y) const {
    return isPointInPolygon(x, y);
}

bool VectorPolygon::hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance) const {
    for (size_t i = 0; i < vertices_.size(); ++i) {
        double dx = x - vertices_[i].first;
        double dy = y - vertices_[i].second;
        if (sqrt(dx * dx + dy * dy) <= tolerance) {
            controlPointIndex = static_cast<int>(i);
            return true;
        }
    }
    return false;
}

void VectorPolygon::startDrag(double x, double y) {
    dragStartX_ = x;
    dragStartY_ = y;
    originalVertices_ = vertices_;
}

void VectorPolygon::updateDrag(double x, double y, bool shiftPressed) {
    double dx = x - dragStartX_;
    double dy = y - dragStartY_;
    
    for (size_t i = 0; i < vertices_.size(); ++i) {
        vertices_[i].first = originalVertices_[i].first + dx;
        vertices_[i].second = originalVertices_[i].second + dy;
    }
}

void VectorPolygon::endDrag() {
    // 多边形拖拽结束无需特殊处理
}

void VectorPolygon::moveControlPoint(int index, double x, double y) {
    if (index >= 0 && index < static_cast<int>(vertices_.size())) {
        vertices_[index].first = x;
        vertices_[index].second = y;
    }
}

VectorShape::ShapeData VectorPolygon::toShapeData() const {
    ShapeData data(SelectionShape::Polygon);
    data.polygonVertices = vertices_;
    return data;
}

std::string VectorPolygon::serialize() const {
    std::string result;
    for (size_t i = 0; i < vertices_.size(); ++i) {
        if (i > 0) result += ";";
        result += std::to_string(vertices_[i].first) + "," + std::to_string(vertices_[i].second);
    }
    return result;
}

void VectorPolygon::deserialize(const std::string& data) {
    vertices_.clear();
    if (data.empty()) return;
    
    size_t start = 0;
    size_t end = data.find(';');
    
    while (true) {
        std::string vertex = (end == std::string::npos) ? data.substr(start) : data.substr(start, end - start);
        
        size_t commaPos = vertex.find(',');
        if (commaPos != std::string::npos) {
            double x = std::stod(vertex.substr(0, commaPos));
            double y = std::stod(vertex.substr(commaPos + 1));
            vertices_.emplace_back(x, y);
        }
        
        if (end == std::string::npos) break;
        start = end + 1;
        end = data.find(';', start);
    }
}

//=============================================================================
// PointCloudSelector 实现 - 模块化的点云选择逻辑
//=============================================================================

PointCloudSelector::PointCloudSelector(vtkRenderer* renderer, vtkPolyData* pointData)
    : renderer_(renderer), originalPointData_(pointData), occlusionDetectionEnabled_(true), 
      useHardwareSelection_(true), useHybridSelection_(true) {
}

std::vector<vtkIdType> PointCloudSelector::selectPointsByShapes(const std::vector<VectorShape*>& shapes) {
    if (!renderer_ || !originalPointData_ || shapes.empty()) {
        return {};
    }
    
    // 使用硬件选择
    if (useHardwareSelection_ && occlusionDetectionEnabled_) {
        // 暂时禁用，使用改进的软件选择
        // return selectVisiblePointsHardware(shapes);
    }
    
    // 原有的软件选择方法
    std::vector<vtkIdType> candidatePoints = collectCandidatePoints(shapes);
    
    if (candidatePoints.empty()) {
        return {};
    }
    
    std::vector<vtkIdType> visiblePoints;
    if (occlusionDetectionEnabled_) {
        // 收集屏幕位置和相机距离信息
        std::map<vtkIdType, std::pair<double, double>> screenPositions;
        std::map<vtkIdType, double> distancesToCamera;
        
        vtkCamera* camera = renderer_->GetActiveCamera();
        double cameraPos[3];
        camera->GetPosition(cameraPos);
        
        vtkPoints* points = originalPointData_->GetPoints();
        for (vtkIdType pointId : candidatePoints) {
            double point[3];
            points->GetPoint(pointId, point);
            
            // 计算屏幕位置
            double screenPoint[3];
            renderer_->SetWorldPoint(point[0], point[1], point[2], 1.0);
            renderer_->WorldToDisplay();
            renderer_->GetDisplayPoint(screenPoint);
            screenPositions[pointId] = std::make_pair(screenPoint[0], screenPoint[1]);
            
            // 计算到相机的距离
            double distance = sqrt(pow(point[0] - cameraPos[0], 2) + 
                                 pow(point[1] - cameraPos[1], 2) + 
                                 pow(point[2] - cameraPos[2], 2));
            distancesToCamera[pointId] = distance;
        }
        
        visiblePoints = filterOccludedPoints(candidatePoints, screenPositions, distancesToCamera);
    } else {
        visiblePoints = candidatePoints;
    }
    
    // 添加到已选中点列表
    addSelectedPoints(visiblePoints);
    
    qDebug() << "点云选择完成：候选点" << candidatePoints.size() << "个，可见点" << visiblePoints.size() << "个，总选中" << selectedPointIds_.size() << "个";
    
    // 调试信息：如果选中率过低，输出警告
    double selectionRate = (double)visiblePoints.size() / candidatePoints.size();
    if (selectionRate < 0.7) {
        qDebug() << QString("警告：选中率较低 (%1%)，可能存在遮挡检测问题").arg(selectionRate * 100, 0, 'f', 1);
    }
    
    return visiblePoints;
}

std::vector<vtkIdType> PointCloudSelector::collectCandidatePoints(const std::vector<VectorShape*>& shapes) {
    std::vector<vtkIdType> candidatePoints;
    vtkPoints* points = originalPointData_->GetPoints();
    
    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
        double point[3];
        points->GetPoint(i, point);
        
        // 将3D点投影到屏幕坐标
        double screenPoint[3];
        renderer_->SetWorldPoint(point[0], point[1], point[2], 1.0);
        renderer_->WorldToDisplay();
        renderer_->GetDisplayPoint(screenPoint);
        
        // 检查是否在任何形状内
        if (isPointInShapes(screenPoint[0], screenPoint[1], shapes)) {
            candidatePoints.push_back(i);
        }
    }
    
    return candidatePoints;
}

bool PointCloudSelector::isPointInShapes(double screenX, double screenY, const std::vector<VectorShape*>& shapes) {
    for (const auto& shape : shapes) {
        if (shape->containsPoint(screenX, screenY)) {
            return true;
        }
    }
    return false;
}

std::vector<vtkIdType> PointCloudSelector::filterOccludedPoints(const std::vector<vtkIdType>& candidatePoints,
                                                               const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                                                               const std::map<vtkIdType, double>& distancesToCamera) {
    if (candidatePoints.empty()) return {};
    
    // 按距离排序（从近到远）
    std::vector<vtkIdType> sortedPoints = candidatePoints;
    std::sort(sortedPoints.begin(), sortedPoints.end(), 
              [&distancesToCamera](vtkIdType a, vtkIdType b) {
                  return distancesToCamera.at(a) < distancesToCamera.at(b);
              });
    
    // 基于深度缓冲的遮挡检测
    std::vector<vtkIdType> visiblePoints;
    double pixelThreshold = 4.0; // 减小像素距离阈值，提高精度
    
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
                    
                    // 改进的遮挡判断：考虑点的密度和角度
                    double depthDiff = pointDistance - existingDistance;
                    double relativeDepthDiff = depthDiff / existingDistance;
                    
                    // 计算屏幕距离
                    double screenDist = sqrt(dx * dx + dy * dy);
                    
                    // 动态阈值：距离越近，阈值越大（允许更多倾斜）
                    double dynamicThreshold = 0.02; // 基础阈值
                    
                    // 如果点很近（在3像素内），使用更宽松的阈值
                    if (screenDist < 3.0) {
                        dynamicThreshold = 0.05; // 允许5%的深度差异
                    } else if (screenDist < pixelThreshold * 0.5) {
                        dynamicThreshold = 0.03; // 允许3%的深度差异
                    }
                    
                    // 只有当深度差异超过动态阈值时才认为被遮挡
                    if (existingDistance < pointDistance && relativeDepthDiff > dynamicThreshold) {
                        // 额外检查：如果深度差异很小，可能是同一表面
                        if (depthDiff < 0.5 && screenDist < 2.0) {
                            // 很可能是同一表面的相邻点，不算遮挡
                            continue;
                        }
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

void PointCloudSelector::highlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds) {
    if (!renderer_ || !originalPointData_) return;
    
    // 注意：参数selectedPointIds被忽略，始终使用成员变量selectedPointIds_
    // 这样可以保持累积选择的效果

    // 获取原始点云的颜色数据或标量数据
    vtkUnsignedCharArray* originalColors = vtkUnsignedCharArray::SafeDownCast(
        originalPointData_->GetPointData()->GetScalars());

    // 如果没有颜色数据，尝试从标量数据创建颜色
    if (!originalColors) {
        vtkDataArray* scalars = originalPointData_->GetPointData()->GetScalars();
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
            originalPointData_->GetPointData()->SetScalars(colors);
            originalColors = colors;

            qDebug() << "已创建颜色数据，点数:" << colors->GetNumberOfTuples();
        } else {
            qDebug() << "原始点云既没有颜色数据也没有标量数据";
            return;
        }
    }

    // 保存原始颜色（如果还没有保存）
    if (originalColorBackup_.empty()) {
        originalColorBackup_.resize(originalColors->GetNumberOfTuples() * 3);
        for (vtkIdType i = 0; i < originalColors->GetNumberOfTuples(); ++i) {
            unsigned char* color = originalColors->GetPointer(i * 3);
            originalColorBackup_[i * 3] = color[0];
            originalColorBackup_[i * 3 + 1] = color[1];
            originalColorBackup_[i * 3 + 2] = color[2];
        }
        qDebug() << "已保存原始颜色备份，点数:" << originalColors->GetNumberOfTuples();
    }

    // 先恢复所有点为原色
    for (vtkIdType i = 0; i < originalColors->GetNumberOfTuples(); ++i) {
        unsigned char* color = originalColors->GetPointer(i * 3);
        color[0] = originalColorBackup_[i * 3];
        color[1] = originalColorBackup_[i * 3 + 1];
        color[2] = originalColorBackup_[i * 3 + 2];
    }

    // 将选中的点设置为红色（使用累积的选中点列表）
    for (vtkIdType id : selectedPointIds_) {
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
    renderer_->GetRenderWindow()->Render();

    qDebug() << "已高亮" << selectedPointIds_.size() << "个点";
}

void PointCloudSelector::clearAllSelectedPoints() {
    if (!renderer_ || !originalPointData_ || originalColorBackup_.empty()) return;
    
    // 获取原始点云的颜色数据
    vtkUnsignedCharArray* originalColors = vtkUnsignedCharArray::SafeDownCast(
        originalPointData_->GetPointData()->GetScalars());
    
    if (!originalColors) return;
    
    // 恢复所有点的原始颜色
    for (vtkIdType i = 0; i < originalColors->GetNumberOfTuples(); ++i) {
        unsigned char* color = originalColors->GetPointer(i * 3);
        color[0] = originalColorBackup_[i * 3];
        color[1] = originalColorBackup_[i * 3 + 1];
        color[2] = originalColorBackup_[i * 3 + 2];
    }
    
    // 标记颜色数据已修改
    originalColors->Modified();
    
    // 清空选中点列表
    selectedPointIds_.clear();
    
    // 刷新渲染
    renderer_->GetRenderWindow()->Render();
    
    qDebug() << "已清除所有选中点";
}

void PointCloudSelector::addSelectedPoints(const std::vector<vtkIdType>& points) {
    // 使用set来去重
    std::set<vtkIdType> uniquePoints(selectedPointIds_.begin(), selectedPointIds_.end());
    uniquePoints.insert(points.begin(), points.end());
    
    // 更新选中点列表
    selectedPointIds_.assign(uniquePoints.begin(), uniquePoints.end());
}

double PointCloudSelector::calculateScreenDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool PointCloudSelector::isPointOccluded(vtkIdType pointId, const std::vector<vtkIdType>& frontPoints, 
                                        const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                                        double occlusionThreshold) {
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
        double distance = calculateScreenDistance(pointX, pointY, frontX, frontY);
        
        // 如果距离小于阈值，认为被遮挡
        if (distance < occlusionThreshold) {
            return true;
        }
    }
    
    return false;
}

// 使用硬件选择器进行可见点选择
std::vector<vtkIdType> PointCloudSelector::selectVisiblePointsHardware(const std::vector<VectorShape*>& shapes) {
    if (!renderer_ || !originalPointData_ || shapes.empty()) {
        return {};
    }
    
    qDebug() << "开始硬件选择（优化版）...";
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // 获取渲染窗口
    vtkRenderWindow* renderWindow = renderer_->GetRenderWindow();
    if (!renderWindow) {
        qDebug() << "错误：无法获取渲染窗口";
        return {};
    }
    
    // 首先收集候选点（在选择框内的点）
    std::vector<vtkIdType> candidatePoints = collectCandidatePoints(shapes);
    qDebug() << "候选点数：" << candidatePoints.size();
    
    if (candidatePoints.empty()) {
        return {};
    }
    
    // 确保场景已经渲染
    renderWindow->Render();
    
    // 方案2：改进的硬件选择，对每个形状单独处理
    std::vector<vtkIdType> visiblePoints;
    std::set<vtkIdType> processedPoints; // 避免重复处理
    
    // 获取点云actor以获取渲染属性
    vtkActor* pointCloudActor = nullptr;
    vtkActorCollection* actors = renderer_->GetActors();
    actors->InitTraversal();
    vtkActor* actor = nullptr;
    while ((actor = actors->GetNextActor())) {
        if (actor->GetMapper() && actor->GetMapper()->GetInput() == originalPointData_) {
            pointCloudActor = actor;
            break;
        }
    }
    
    double pointSize = 1.0;
    if (pointCloudActor && pointCloudActor->GetProperty()) {
        pointSize = pointCloudActor->GetProperty()->GetPointSize();
        qDebug() << "点渲染大小：" << pointSize;
    }
    
    // 对每个形状分别处理，以提高精度
    for (const auto* shape : shapes) {
        // 收集该形状内的候选点
        std::vector<vtkIdType> shapeCandidate;
        vtkPoints* points = originalPointData_->GetPoints();
        
        for (vtkIdType pointId : candidatePoints) {
            if (processedPoints.find(pointId) != processedPoints.end()) {
                continue; // 已处理过
            }
            
            double point[3];
            points->GetPoint(pointId, point);
            
            double screenPoint[3];
            renderer_->SetWorldPoint(point[0], point[1], point[2], 1.0);
            renderer_->WorldToDisplay();
            renderer_->GetDisplayPoint(screenPoint);
            
            // 只检查当前形状
            std::vector<VectorShape*> singleShape = {const_cast<VectorShape*>(shape)};
            if (isPointInShapes(screenPoint[0], screenPoint[1], singleShape)) {
                shapeCandidate.push_back(pointId);
            }
        }
        
        if (shapeCandidate.empty()) continue;
        
        // 计算该形状候选点的屏幕边界
        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double maxY = std::numeric_limits<double>::lowest();
        
        for (vtkIdType pointId : shapeCandidate) {
            double point[3];
            points->GetPoint(pointId, point);
            
            double screenPoint[3];
            renderer_->SetWorldPoint(point[0], point[1], point[2], 1.0);
            renderer_->WorldToDisplay();
            renderer_->GetDisplayPoint(screenPoint);
            
            minX = std::min(minX, screenPoint[0]);
            minY = std::min(minY, screenPoint[1]);
            maxX = std::max(maxX, screenPoint[0]);
            maxY = std::max(maxY, screenPoint[1]);
        }
        
        // 根据点的渲染大小扩展边界
        double margin = std::max(5.0, pointSize * 2.0);
        minX = std::max(0.0, minX - margin);
        minY = std::max(0.0, minY - margin);
        maxX = maxX + margin;
        maxY = maxY + margin;
        
        // 创建硬件选择器
        vtkSmartPointer<vtkHardwareSelector> selector = vtkSmartPointer<vtkHardwareSelector>::New();
        selector->SetRenderer(renderer_);
        selector->SetFieldAssociation(vtkDataObject::FIELD_ASSOCIATION_POINTS);
        
        // 设置选择区域
        int* windowSize = renderWindow->GetSize();
        unsigned int x0 = static_cast<unsigned int>(minX);
        unsigned int y0 = static_cast<unsigned int>(windowSize[1] - maxY); // Y坐标翻转
        unsigned int x1 = static_cast<unsigned int>(maxX);
        unsigned int y1 = static_cast<unsigned int>(windowSize[1] - minY); // Y坐标翻转
        
        // 确保坐标在窗口范围内
        x0 = std::min(x0, static_cast<unsigned int>(windowSize[0] - 1));
        y0 = std::min(y0, static_cast<unsigned int>(windowSize[1] - 1));
        x1 = std::min(x1, static_cast<unsigned int>(windowSize[0] - 1));
        y1 = std::min(y1, static_cast<unsigned int>(windowSize[1] - 1));
        
        qDebug() << QString("形状 %1 选择区域: (%2,%3) - (%4,%5)")
            .arg(shapes.size()).arg(x0).arg(y0).arg(x1).arg(y1);
        
        // 执行区域选择
        selector->SetArea(x0, y0, x1, y1);
        vtkSmartPointer<vtkSelection> selection = selector->Select();
        
        if (selection) {
            // 收集所有被选中的点
            std::set<vtkIdType> selectedIds;
            unsigned int numNodes = selection->GetNumberOfNodes();
            
            for (unsigned int nodeIdx = 0; nodeIdx < numNodes; nodeIdx++) {
                vtkSelectionNode* node = selection->GetNode(nodeIdx);
                vtkIdTypeArray* selectionIds = vtkIdTypeArray::SafeDownCast(
                    node->GetSelectionList());
                
                if (selectionIds) {
                    for (vtkIdType i = 0; i < selectionIds->GetNumberOfTuples(); i++) {
                        selectedIds.insert(selectionIds->GetValue(i));
                    }
                }
            }
            
            qDebug() << "该形状硬件选择返回了" << selectedIds.size() << "个点";
            
            // 过滤出既在选择框内又被硬件选择的点
            int missedCount = 0;
            for (vtkIdType pointId : shapeCandidate) {
                if (selectedIds.find(pointId) != selectedIds.end()) {
                    visiblePoints.push_back(pointId);
                    processedPoints.insert(pointId);
                } else {
                    missedCount++;
                }
            }
            
            // 如果有较多点被遗漏，尝试使用混合方法
            if (useHybridSelection_ && missedCount > shapeCandidate.size() * 0.1) {
                qDebug() << QString("检测到 %1 个点可能被遗漏，启用混合选择").arg(missedCount);
                
                // 对遗漏的点进行额外检查
                for (vtkIdType pointId : shapeCandidate) {
                    if (processedPoints.find(pointId) != processedPoints.end()) {
                        continue; // 已处理
                    }
                    
                    double point[3];
                    points->GetPoint(pointId, point);
                    
                    // 检查周围是否有已选中的点
                    bool hasNearbySelected = false;
                    double checkRadius = 5.0; // 像素
                    
                    double screenPoint[3];
                    renderer_->SetWorldPoint(point[0], point[1], point[2], 1.0);
                    renderer_->WorldToDisplay();
                    renderer_->GetDisplayPoint(screenPoint);
                    
                    // 检查周围已选中的点
                    for (vtkIdType selectedId : visiblePoints) {
                        double selectedPoint[3];
                        points->GetPoint(selectedId, selectedPoint);
                        
                        double selectedScreen[3];
                        renderer_->SetWorldPoint(selectedPoint[0], selectedPoint[1], selectedPoint[2], 1.0);
                        renderer_->WorldToDisplay();
                        renderer_->GetDisplayPoint(selectedScreen);
                        
                        double dist = sqrt(pow(screenPoint[0] - selectedScreen[0], 2) + 
                                         pow(screenPoint[1] - selectedScreen[1], 2));
                        
                        if (dist < checkRadius) {
                            hasNearbySelected = true;
                            break;
                        }
                    }
                    
                    // 如果周围有选中的点，且在相似深度，则也选中该点
                    if (hasNearbySelected) {
                        visiblePoints.push_back(pointId);
                        processedPoints.insert(pointId);
                    }
                }
            }
        }
    }
    
    // 添加到已选中点列表
    addSelectedPoints(visiblePoints);
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    qDebug() << "硬件选择完成，用时：" << duration.count() << "ms";
    qDebug() << "候选点：" << candidatePoints.size() << "，可见点：" << visiblePoints.size();
    
    return visiblePoints;
}

//=============================================================================
// Selector 实现
//=============================================================================

Selector::Selector()
    : currentDrawingShape_(SelectionShape::Rectangle)
    , isDrawing_(false)
    , startX_(0)
    , startY_(0)
    , currentX_(0)
    , currentY_(0)
    , selectedShape_(nullptr)
    , currentOperation_(EditOperation::None)
    , selectedControlPoint_(-1)
    , isDragging_(false)
    , lastMouseX_(0)
    , lastMouseY_(0)
    , shapeStateBeforeDrag_("")
    , viewLocked_(false)
{
    // 初始化状态实例
    drawingState_ = std::make_unique<DrawingState>();
    editingState_ = std::make_unique<EditingState>();
    polygonDrawingState_ = std::make_unique<PolygonDrawingState>();
    
    // 默认为编辑状态
    currentState_ = std::make_unique<EditingState>();
    
    // 初始化控制点显示
    controlPoints_ = vtkSmartPointer<vtkPoints>::New();
    controlPolyData_ = vtkSmartPointer<vtkPolyData>::New();
    controlPolyData_->SetPoints(controlPoints_);
    controlMapper_ = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    controlMapper_->SetInputData(controlPolyData_);
    controlActor_ = vtkSmartPointer<vtkActor2D>::New();
    controlActor_->SetMapper(controlMapper_);
    controlActor_->GetProperty()->SetColor(1.0, 1.0, 0.0); // 黄色控制点
    controlActor_->GetProperty()->SetPointSize(8.0);
    
    // 初始化当前形状显示
    currentShapePoints_ = vtkSmartPointer<vtkPoints>::New();
    currentShapePolyData_ = vtkSmartPointer<vtkPolyData>::New();
    currentShapePolyData_->SetPoints(currentShapePoints_);
    currentShapeMapper_ = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    currentShapeMapper_->SetInputData(currentShapePolyData_);
    currentShapeActor_ = vtkSmartPointer<vtkActor2D>::New();
    currentShapeActor_->SetMapper(currentShapeMapper_);
    currentShapeActor_->GetProperty()->SetColor(1.0, 1.0, 0.0); // 黄色，表示正在绘制
    currentShapeActor_->GetProperty()->SetLineWidth(2.0);
    
    // 设置2D坐标系统为屏幕坐标
    vtkSmartPointer<vtkCoordinate> coord = vtkSmartPointer<vtkCoordinate>::New();
    coord->SetCoordinateSystem(0); // VTK_DISPLAY = 0，屏幕坐标
    controlMapper_->SetTransformCoordinate(coord);
    currentShapeMapper_->SetTransformCoordinate(coord);
}

Selector::~Selector()
{
}

void Selector::SetRenderer(vtkRenderer* ren)
{
    renderer_ = ren;
    if (renderer_) {
        renderer_->AddActor2D(controlActor_);
        renderer_->AddActor2D(currentShapeActor_);
        controlActor_->SetVisibility(0);
        currentShapeActor_->SetVisibility(0);
        
        // 如果已有点云数据，创建PointCloudSelector
        if (pointCloudSelector_ && pointCloudSelector_->getSelectedPoints().empty()) {
            pointCloudSelector_->setRenderer(renderer_);
        }
    }
}

void Selector::SetPointCloudData(vtkPolyData* pointData)
{
    // 创建或更新PointCloudSelector
    if (!pointCloudSelector_) {
        pointCloudSelector_ = std::make_unique<PointCloudSelector>(renderer_, pointData);
    } else {
        pointCloudSelector_->setPointCloudData(pointData);
        if (renderer_) {
            pointCloudSelector_->setRenderer(renderer_);
        }
    }
}

void Selector::EnableDrawingMode(bool enable)
{
    if (enable) {
        // 切换到绘制状态
        if (currentDrawingShape_ == SelectionShape::Polygon) {
            SetState(std::make_unique<PolygonDrawingState>());
        } else {
            SetState(std::make_unique<DrawingState>());
        }
    } else {
        // 切换到编辑状态
        SetState(std::make_unique<EditingState>());
        ClearCurrentDrawing();
    }
}

bool Selector::IsDrawingModeEnabled() const
{
    return (currentState_->getStateName() == "Drawing" || 
            currentState_->getStateName() == "PolygonDrawing");
}

void Selector::EnableOcclusionDetection(bool enable)
{
    if (pointCloudSelector_) {
        pointCloudSelector_->setOcclusionDetectionEnabled(enable);
    }
}

bool Selector::IsOcclusionDetectionEnabled() const
{
    return pointCloudSelector_ ? pointCloudSelector_->getSelectedPoints().empty() : true;
}

void Selector::SetCurrentDrawingShape(SelectionShape shape)
{
    currentDrawingShape_ = shape;
    
    // 如果当前是绘制模式，需要根据形状类型切换状态
    if (IsDrawingModeEnabled()) {
        if (shape == SelectionShape::Polygon) {
            SetState(std::make_unique<PolygonDrawingState>());
        } else {
            SetState(std::make_unique<DrawingState>());
        }
    }
}

void Selector::SetState(std::unique_ptr<SelectionState> state)
{
    currentState_ = std::move(state);
}

void Selector::ClearCanvas()
{
    // 移除所有矢量图形的actors
    if (renderer_) {
        for (auto& actor : vectorShapeActors_) {
            renderer_->RemoveActor2D(actor);
        }
    }
    
    // 清空容器
    vectorShapes_.clear();
    vectorShapeActors_.clear();
    vectorShapeMappers_.clear();
    vectorShapePolyDatas_.clear();
    vectorShapePointsList_.clear();
    
    // 清空命令历史
    while (!commandHistory_.empty()) {
        commandHistory_.pop();
    }
    
    DeselectAllShapes();
    controlActor_->SetVisibility(0);
    
    if (renderer_) {
        renderer_->GetRenderWindow()->Render();
    }
    qDebug() << "画布已清空";
}

void Selector::ClearCurrentDrawing()
{
    isDrawing_ = false;
    currentPolygonVertices_.clear();
    currentShapeActor_->SetVisibility(0);
    
    if (renderer_) {
        renderer_->GetRenderWindow()->Render();
    }
    
    qDebug() << "已清除当前绘制状态";
}

void Selector::UpdateVectorShapeDisplay()
{
    if (!renderer_) return;
    
    // 先移除旧的actors
    for (auto& actor : vectorShapeActors_) {
        renderer_->RemoveActor2D(actor);
    }
    
    // 清空旧的显示容器
    vectorShapeActors_.clear();
    vectorShapeMappers_.clear();
    vectorShapePolyDatas_.clear();
    vectorShapePointsList_.clear();
    
    if (vectorShapes_.empty()) {
        controlActor_->SetVisibility(0);
        if (renderer_) {
            renderer_->GetRenderWindow()->Render();
        }
        return;
    }
    
    // 为每个形状创建独立的actor
    for (size_t i = 0; i < vectorShapes_.size(); ++i) {
        auto& shape = vectorShapes_[i];
        
        // 创建独立的VTK对象
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto polyData = vtkSmartPointer<vtkPolyData>::New();
        auto mapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
        auto actor = vtkSmartPointer<vtkActor2D>::New();
        
        polyData->SetPoints(points);
        mapper->SetInputData(polyData);
        actor->SetMapper(mapper);
        
        // 设置2D坐标系统
        vtkSmartPointer<vtkCoordinate> coord = vtkSmartPointer<vtkCoordinate>::New();
        coord->SetCoordinateSystem(0); // VTK_DISPLAY = 0，屏幕坐标
        mapper->SetTransformCoordinate(coord);
        
        // 绘制形状
        shape->draw(renderer_, actor, mapper, polyData, points);
        
        // 添加到渲染器
        renderer_->AddActor2D(actor);
        actor->SetVisibility(1);
        
        // 保存到容器
        vectorShapeActors_.push_back(actor);
        vectorShapeMappers_.push_back(mapper);
        vectorShapePolyDatas_.push_back(polyData);
        vectorShapePointsList_.push_back(points);
    }
    
    // 绘制控制点
    DrawControlPoints();
    
    if (renderer_) {
        renderer_->GetRenderWindow()->Render();
    }
}

void Selector::DrawControlPoints()
{
    if (!selectedShape_) {
        controlActor_->SetVisibility(0);
        return;
    }
    
    std::vector<std::pair<double, double>> controlPts;
    
    if (selectedShape_->getType() == SelectionShape::Rectangle) {
        VectorRectangle* rect = static_cast<VectorRectangle*>(selectedShape_);
        auto rectShape = rect->toShapeData();
        controlPts = {
            {rectShape.rect.x1, rectShape.rect.y1}, {rectShape.rect.x2, rectShape.rect.y1}, 
            {rectShape.rect.x2, rectShape.rect.y2}, {rectShape.rect.x1, rectShape.rect.y2}, // 四个角
            {(rectShape.rect.x1 + rectShape.rect.x2) / 2, rectShape.rect.y1}, 
            {rectShape.rect.x2, (rectShape.rect.y1 + rectShape.rect.y2) / 2}, 
            {(rectShape.rect.x1 + rectShape.rect.x2) / 2, rectShape.rect.y2}, 
            {rectShape.rect.x1, (rectShape.rect.y1 + rectShape.rect.y2) / 2}  // 四个边中点
        };
    } else if (selectedShape_->getType() == SelectionShape::Circle) {
        VectorCircle* circle = static_cast<VectorCircle*>(selectedShape_);
        auto circleShape = circle->toShapeData();
        controlPts = {
            {circleShape.circle.centerX + circleShape.circle.radius, circleShape.circle.centerY}, // 右
            {circleShape.circle.centerX, circleShape.circle.centerY - circleShape.circle.radius}, // 上
            {circleShape.circle.centerX - circleShape.circle.radius, circleShape.circle.centerY}, // 左
            {circleShape.circle.centerX, circleShape.circle.centerY + circleShape.circle.radius}  // 下
        };
    } else if (selectedShape_->getType() == SelectionShape::Polygon) {
        VectorPolygon* polygon = static_cast<VectorPolygon*>(selectedShape_);
        auto polygonShape = polygon->toShapeData();
        controlPts = polygonShape.polygonVertices;
    }
    
    if (controlPts.empty()) {
        controlActor_->SetVisibility(0);
        return;
    }
    
    controlPoints_->SetNumberOfPoints(controlPts.size());
    for (size_t i = 0; i < controlPts.size(); ++i) {
        controlPoints_->SetPoint(i, controlPts[i].first, controlPts[i].second, 0);
    }
    
    // 创建顶点
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    for (size_t i = 0; i < controlPts.size(); ++i) {
        vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
        vertex->GetPointIds()->SetId(0, i);
        vertices->InsertNextCell(vertex);
    }
    controlPolyData_->SetVerts(vertices);
    
    controlPoints_->Modified();
    controlPolyData_->Modified();
    controlActor_->SetVisibility(1);
}

VectorShape* Selector::GetShapeAtPosition(double x, double y)
{
    for (auto& shape : vectorShapes_) {
        if (shape->hitTest(x, y)) {
            return shape.get();
        }
    }
    return nullptr;
}

void Selector::SelectShape(VectorShape* shape)
{
    // 如果点击的是已选中的图形，不需要重新选择
    if (selectedShape_ == shape) {
        return;
    }
    
    DeselectAllShapes();
    if (shape) {
        selectedShape_ = shape;
        shape->setSelected(true);
        UpdateVectorShapeDisplay();
        
        if (cursorCallback_) {
            cursorCallback_(Qt::SizeAllCursor);
        }
    }
}

void Selector::DeselectAllShapes()
{
    if (selectedShape_) {
        selectedShape_->setSelected(false);
        selectedShape_ = nullptr;
        
        // 更新显示以反映选择状态的变化
        UpdateVectorShapeDisplay();
    }
    
    controlActor_->SetVisibility(0);
    
    if (cursorCallback_) {
        cursorCallback_(Qt::ArrowCursor);
    }
}

void Selector::OnLeftButtonDown()
{
    if (!IsDrawingModeEnabled()) {
        // 编辑模式：处理图形选择和编辑
        int x, y;
        this->GetInteractor()->GetEventPosition(x, y);
        
        // 检查是否点击了控制点
        if (selectedShape_) {
            int controlPointIndex = -1;
            if (selectedShape_->hitTestControlPoint(x, y, controlPointIndex)) {
                currentOperation_ = EditOperation::EditVertex;
                selectedControlPoint_ = controlPointIndex;
                isDragging_ = true;
                lastMouseX_ = x;
                lastMouseY_ = y;
                
                // 记录编辑前的状态
                shapeStateBeforeDrag_ = selectedShape_->serialize();
                
                if (cursorCallback_) {
                    cursorCallback_(Qt::PointingHandCursor);
                }
                return;
            }
        }
        
        // 检查是否点击了图形
        VectorShape* clickedShape = GetShapeAtPosition(x, y);
        if (clickedShape) {
            if (clickedShape != selectedShape_) {
                SelectShape(clickedShape);
            }
            
            // 开始拖拽图形
            currentOperation_ = EditOperation::Move;
            isDragging_ = true;
            lastMouseX_ = x;
            lastMouseY_ = y;
            
            // 记录拖拽前的状态
            shapeStateBeforeDrag_ = selectedShape_->serialize();
            selectedShape_->startDrag(x, y);
            
            if (cursorCallback_) {
                cursorCallback_(Qt::SizeAllCursor);
            }
        } else {
            // 点击空白区域，取消选择
            DeselectAllShapes();
            
            if (!viewLocked_) {
                vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
            }
        }
        return;
    }
    
    // 绘制模式：使用状态模式委托处理
    int x, y;
    this->GetInteractor()->GetEventPosition(x, y);
    
    if (currentState_) {
        currentState_->handleMouseDown(this, x, y);
    }
}

void Selector::OnLeftButtonUp()
{
    if (!IsDrawingModeEnabled()) {
        // 编辑模式：结束拖拽操作
        if (isDragging_) {
            if (selectedShape_) {
                selectedShape_->endDrag();
                
                // 记录拖拽后的状态并创建命令
                std::string stateAfterDrag = selectedShape_->serialize();
                if (stateAfterDrag != shapeStateBeforeDrag_) {
                    std::string description = (currentOperation_ == EditOperation::Move) ? "移动图形" : "编辑图形";
                    auto command = std::make_unique<StateChangeCommand>(
                        selectedShape_, 
                        shapeStateBeforeDrag_, 
                        stateAfterDrag, 
                        description);
                    
                    // 不调用ExecuteCommand，因为操作已经执行了，只需要记录
                    commandHistory_.push(std::move(command));
                    
                    // 限制历史记录大小
                    if (commandHistory_.size() > MAX_HISTORY_SIZE) {
                        std::stack<std::unique_ptr<Command>> newHistory;
                        size_t keepCount = std::min(commandHistory_.size(), MAX_HISTORY_SIZE);
                        for (size_t i = 0; i < keepCount; ++i) {
                            if (!commandHistory_.empty()) {
                                newHistory.push(std::move(commandHistory_.top()));
                                commandHistory_.pop();
                            }
                        }
                        commandHistory_ = std::move(newHistory);
                    }
                }
                
                UpdateVectorShapeDisplay();
            }
            
            isDragging_ = false;
            currentOperation_ = EditOperation::None;
            selectedControlPoint_ = -1;
            shapeStateBeforeDrag_.clear();
            
            if (cursorCallback_) {
                if (selectedShape_) {
                    cursorCallback_(Qt::SizeAllCursor);
                } else {
                    cursorCallback_(Qt::ArrowCursor);
                }
            }
        } else if (!viewLocked_) {
            vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
        }
        return;
    }
    
    // 绘制模式：使用状态模式委托处理
    if (currentState_) {
        currentState_->handleMouseUp(this);
    }
}

void Selector::OnRightButtonDown()
{
    if (currentState_) {
        currentState_->handleRightClick(this);
    }
}

void Selector::OnKeyPress()
{
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    bool ctrlPressed = (rwi->GetControlKey() != 0);
    
    // 全局快捷键
    if (ctrlPressed && key == "z") {
        Undo();
        return;
    }
    
    if (key == "Delete") {
        DeleteSelectedShape();
        return;
    }
    
    if (currentState_) {
        currentState_->handleKeyPress(this, key, ctrlPressed);
    }
}

void Selector::OnMouseMove()
{
    int x, y;
    this->GetInteractor()->GetEventPosition(x, y);
    
    if (!IsDrawingModeEnabled()) {
        // 编辑模式：处理拖拽操作
        if (isDragging_ && selectedShape_) {
            // 检查是否按下了Shift键
            bool shiftPressed = (this->GetInteractor()->GetShiftKey() != 0);
            
            if (currentOperation_ == EditOperation::Move) {
                selectedShape_->updateDrag(x, y, shiftPressed);
                UpdateVectorShapeDisplay();
            } else if (currentOperation_ == EditOperation::EditVertex && selectedControlPoint_ >= 0) {
                selectedShape_->moveControlPoint(selectedControlPoint_, x, y);
                UpdateVectorShapeDisplay();
            }
            
            lastMouseX_ = x;
            lastMouseY_ = y;
        } else if (!viewLocked_) {
            vtkInteractorStyleTrackballCamera::OnMouseMove();
        }
        return;
    }
    
    // 绘制模式：使用状态模式委托处理
    if (currentState_) {
        currentState_->handleMouseMove(this, x, y);
    }
}

void Selector::DrawCurrentShape()
{
    switch (currentDrawingShape_) {
        case SelectionShape::Rectangle:
            DrawRectangle(std::min(startX_, currentX_), std::min(startY_, currentY_), 
                         std::max(startX_, currentX_), std::max(startY_, currentY_));
            break;
        case SelectionShape::Circle: {
            double centerX = (startX_ + currentX_) / 2.0;
            double centerY = (startY_ + currentY_) / 2.0;
            double radius = sqrt(pow(currentX_ - startX_, 2) + pow(currentY_ - startY_, 2)) / 2.0;
            DrawCircle(centerX, centerY, radius);
            break;
        }
        case SelectionShape::Polygon: {
            std::vector<std::pair<double, double>> vertices;
            for (const auto& v : currentPolygonVertices_) {
                vertices.emplace_back(v.first, v.second);
            }
            // 在绘制时添加当前鼠标位置作为临时顶点
            vertices.emplace_back(currentX_, currentY_);
            DrawPolygon(vertices, false);
            break;
        }
    }
}

void Selector::DrawRectangle(double x1, double y1, double x2, double y2)
{
    if (!renderer_) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer_->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 确保坐标在屏幕范围内
    x1 = std::max(0.0, std::min((double)size[0], x1));
    x2 = std::max(0.0, std::min((double)size[0], x2));
    y1 = std::max(0.0, std::min((double)size[1], y1));
    y2 = std::max(0.0, std::min((double)size[1], y2));
    
    // 重置点和线为矩形结构
    currentShapePoints_->SetNumberOfPoints(4);
    
    // 更新矩形顶点（使用屏幕坐标）
    currentShapePoints_->SetPoint(0, x2, y1, 0);
    currentShapePoints_->SetPoint(1, x1, y1, 0);
    currentShapePoints_->SetPoint(2, x1, y2, 0);
    currentShapePoints_->SetPoint(3, x2, y2, 0);
    
    // 更新线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < 4; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i + 1) % 4);
        lines->InsertNextCell(line);
    }
    currentShapePolyData_->SetLines(lines);
    
    currentShapePoints_->Modified();
    currentShapePolyData_->Modified();
    renderer_->GetRenderWindow()->Render();
}

void Selector::DrawCircle(double centerX, double centerY, double radius)
{
    if (!renderer_) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer_->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 确保圆心在屏幕范围内
    centerX = std::max(radius, std::min(size[0] - radius, centerX));
    centerY = std::max(radius, std::min(size[1] - radius, centerY));
    
    // 生成圆形点集（提高细分数量获得更平滑的圆形）
    const int numPoints = 128; // 从64提升到128
    currentShapePoints_->SetNumberOfPoints(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * M_PI * i / numPoints;
        double x = centerX + radius * cos(angle);
        double y = centerY + radius * sin(angle);
        currentShapePoints_->SetPoint(i, x, y, 0);
    }
    
    // 更新线条连接
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (int i = 0; i < numPoints; ++i) {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, i);
        line->GetPointIds()->SetId(1, (i + 1) % numPoints);
        lines->InsertNextCell(line);
    }
    currentShapePolyData_->SetLines(lines);
    
    currentShapePoints_->Modified();
    currentShapePolyData_->Modified();
    renderer_->GetRenderWindow()->Render();
}

void Selector::DrawPolygon(const std::vector<std::pair<double, double>>& vertices, bool addTemporaryVertex)
{
    if (!renderer_ || vertices.empty()) return;
    
    // 获取渲染窗口大小
    vtkRenderWindow* renderWindow = renderer_->GetRenderWindow();
    int* size = renderWindow->GetSize();
    
    // 设置顶点数量
    currentShapePoints_->SetNumberOfPoints(vertices.size());
    
    // 更新顶点坐标
    for (size_t i = 0; i < vertices.size(); ++i) {
        double x = std::max(0.0, std::min((double)size[0], vertices[i].first));
        double y = std::max(0.0, std::min((double)size[1], vertices[i].second));
        currentShapePoints_->SetPoint(i, x, y, 0);
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
        
        // 连接最后一个点到第一个点形成闭合图形
        // 如果顶点数>=3则显示闭合线
        if (vertices.size() >= 3) {
            vtkSmartPointer<vtkLine> closingLine = vtkSmartPointer<vtkLine>::New();
            closingLine->GetPointIds()->SetId(0, vertices.size() - 1);
            closingLine->GetPointIds()->SetId(1, 0);
            lines->InsertNextCell(closingLine);
        }
    }
    
    currentShapePolyData_->SetLines(lines);
    currentShapePoints_->Modified();
    currentShapePolyData_->Modified();
    renderer_->GetRenderWindow()->Render();
}

void Selector::AddPolygonVertex(int x, int y)
{
    currentPolygonVertices_.emplace_back(x, y);
    
    // 添加第一个顶点时才显示选择框
    if (currentPolygonVertices_.size() == 1) {
        currentShapeActor_->SetVisibility(1);
    }
    
    DrawCurrentShape(); // 使用新的绘制方法
    qDebug() << "添加多边形顶点:" << x << "," << y << "，当前顶点数:" << currentPolygonVertices_.size();
}

void Selector::UndoLastVertex()
{
    if (currentPolygonVertices_.empty()) {
        qDebug() << "没有顶点可以撤销";
        return;
    }
    
    // 删除最后一个顶点
    auto lastVertex = currentPolygonVertices_.back();
    currentPolygonVertices_.pop_back();
    
    qDebug() << "撤销顶点:" << lastVertex.first << "," << lastVertex.second << "，剩余顶点数:" << currentPolygonVertices_.size();
    
    if (currentPolygonVertices_.empty()) {
        // 如果所有顶点都被删除，取消当前绘制
        currentShapeActor_->SetVisibility(0);
        renderer_->GetRenderWindow()->Render();
        
        if (cursorCallback_) {
            cursorCallback_(Qt::ArrowCursor);
        }
        
        qDebug() << "所有顶点已撤销，取消多边形绘制";
    } else {
        // 重新绘制多边形
        DrawCurrentShape();
    }
}

//=============================================================================
// 撤销系统实现
//=============================================================================

void Selector::ExecuteCommand(std::unique_ptr<Command> command)
{
    command->execute();
    
    // 添加到历史记录
    commandHistory_.push(std::move(command));
    
    // 限制历史记录大小
    if (commandHistory_.size() > MAX_HISTORY_SIZE) {
        std::stack<std::unique_ptr<Command>> newHistory;
        for (size_t i = 0; i < MAX_HISTORY_SIZE; ++i) {
            if (!commandHistory_.empty()) {
                newHistory.push(std::move(commandHistory_.top()));
                commandHistory_.pop();
            }
        }
        commandHistory_ = std::move(newHistory);
    }
    
    UpdateVectorShapeDisplay();
}

void Selector::Undo()
{
    if (commandHistory_.empty()) {
        qDebug() << "没有可撤销的操作";
        return;
    }
    
    auto command = std::move(commandHistory_.top());
    commandHistory_.pop();
    
    command->undo();
    qDebug() << "已撤销操作：" << QString::fromStdString(command->getDescription());
    
    // 如果撤销后没有选中的图形，清除选择状态
    if (selectedShape_ && GetShapeIndex(selectedShape_) == SIZE_MAX) {
        DeselectAllShapes();
    } else {
        // 强制更新显示
        UpdateVectorShapeDisplay();
    }
}

void Selector::DeleteSelectedShape()
{
    if (!selectedShape_) {
        qDebug() << "没有选中的图形可删除";
        return;
    }
    
    auto deleteCommand = std::make_unique<DeleteShapeCommand>(this, selectedShape_);
    ExecuteCommand(std::move(deleteCommand));
    
    DeselectAllShapes();
    qDebug() << "已删除选中的图形";
}

bool Selector::ConvertRectangleToPolygon()
{
    // 1. 检查是否有选中的图形且为矩形
    if (!selectedShape_) {
        qDebug() << "没有选中的图形";
        return false;
    }
    
    if (selectedShape_->getType() != SelectionShape::Rectangle) {
        qDebug() << "选中的图形不是矩形，无法转换";
        return false;
    }
    
    // 2. 获取矩形的四个角点
    VectorRectangle* rectangle = static_cast<VectorRectangle*>(selectedShape_);
    auto controlPoints = rectangle->getControlPoints();
    
    // 前4个点是矩形的四个角点：左上、右上、右下、左下
    std::vector<std::pair<double, double>> polygonVertices = {
        controlPoints[0], // 左上角
        controlPoints[1], // 右上角  
        controlPoints[2], // 右下角
        controlPoints[3]  // 左下角
    };
    
    qDebug() << "矩形角点：" 
             << "(" << controlPoints[0].first << "," << controlPoints[0].second << ") "
             << "(" << controlPoints[1].first << "," << controlPoints[1].second << ") "
             << "(" << controlPoints[2].first << "," << controlPoints[2].second << ") "
             << "(" << controlPoints[3].first << "," << controlPoints[3].second << ")";
    
    // 3. 创建新的多边形对象
    auto newPolygon = std::make_unique<VectorPolygon>(polygonVertices);
    
    // 4. 保存矩形的索引位置以便在同一位置插入多边形
    size_t rectangleIndex = GetShapeIndex(selectedShape_);
    
    // 5. 创建组合命令：删除矩形 + 添加多边形
    // 先删除矩形
    auto deleteCommand = std::make_unique<DeleteShapeCommand>(this, selectedShape_);
    ExecuteCommand(std::move(deleteCommand));
    
    // 再在相同位置添加多边形
    auto addCommand = std::make_unique<AddShapeCommand>(this, std::move(newPolygon));
    ExecuteCommand(std::move(addCommand));
    
    // 6. 选中新创建的多边形
    if (rectangleIndex < vectorShapes_.size()) {
        SelectShape(vectorShapes_[rectangleIndex].get());
    }
    
    qDebug() << "矩形已成功转换为多边形";
    return true;
}

void Selector::AddShapeInternal(std::unique_ptr<VectorShape> shape)
{
    vectorShapes_.push_back(std::move(shape));
}

std::unique_ptr<VectorShape> Selector::RemoveShapeInternal(VectorShape* shape)
{
    for (auto it = vectorShapes_.begin(); it != vectorShapes_.end(); ++it) {
        if (it->get() == shape) {
            if (selectedShape_ == shape) {
                selectedShape_ = nullptr;
            }
            auto removedShape = std::move(*it);
            vectorShapes_.erase(it);
            return removedShape;
        }
    }
    return nullptr;
}

void Selector::InsertShapeInternal(std::unique_ptr<VectorShape> shape, size_t index)
{
    if (index >= vectorShapes_.size()) {
        vectorShapes_.push_back(std::move(shape));
    } else {
        vectorShapes_.insert(vectorShapes_.begin() + index, std::move(shape));
    }
}

size_t Selector::GetShapeIndex(VectorShape* shape) const
{
    for (size_t i = 0; i < vectorShapes_.size(); ++i) {
        if (vectorShapes_[i].get() == shape) {
            return i;
        }
    }
    return SIZE_MAX; // 表示未找到
}

//=============================================================================
// 命令类实现
//=============================================================================

// AddShapeCommand 实现
AddShapeCommand::AddShapeCommand(Selector* selector, std::unique_ptr<VectorShape> shape)
    : selector_(selector), shape_(std::move(shape)), executed_(false) {
    shapePtr_ = shape_.get(); // 保存原始指针
}

void AddShapeCommand::execute() {
    if (!executed_) {
        selector_->AddShapeInternal(std::move(shape_));
        executed_ = true;
    }
}

void AddShapeCommand::undo() {
    if (executed_) {
        // 使用保存的指针来查找和移除图形，重新获得所有权
        shape_ = selector_->RemoveShapeInternal(shapePtr_);
        executed_ = false;
    }
}

std::string AddShapeCommand::getDescription() const {
    return "添加图形";
}

// DeleteShapeCommand 实现
DeleteShapeCommand::DeleteShapeCommand(Selector* selector, VectorShape* shape)
    : selector_(selector), originalShapePtr_(shape), executed_(false) {
    originalIndex_ = selector_->GetShapeIndex(shape);
    
    // 创建形状的副本用于撤销时恢复
    switch (shape->getType()) {
        case SelectionShape::Rectangle: {
            auto* rect = static_cast<VectorRectangle*>(shape);
            auto rectShape = rect->toShapeData();
            shape_ = std::make_unique<VectorRectangle>(rectShape.rect.x1, rectShape.rect.y1, 
                                                      rectShape.rect.x2, rectShape.rect.y2);
            break;
        }
        case SelectionShape::Circle: {
            auto* circle = static_cast<VectorCircle*>(shape);
            auto circleShape = circle->toShapeData();
            shape_ = std::make_unique<VectorCircle>(circleShape.circle.centerX, circleShape.circle.centerY, 
                                                   circleShape.circle.radius);
            break;
        }
        case SelectionShape::Polygon: {
            auto* polygon = static_cast<VectorPolygon*>(shape);
            auto polygonShape = polygon->toShapeData();
            shape_ = std::make_unique<VectorPolygon>(polygonShape.polygonVertices);
            break;
        }
    }
}

void DeleteShapeCommand::execute() {
    if (!executed_) {
        // 使用原始形状指针来删除图形
        selector_->RemoveShapeInternal(originalShapePtr_);
        executed_ = true;
    }
}

void DeleteShapeCommand::undo() {
    if (executed_) {
        auto shapeCopy = std::unique_ptr<VectorShape>();
        
        // 重新创建形状
        switch (shape_->getType()) {
            case SelectionShape::Rectangle: {
                auto* rect = static_cast<VectorRectangle*>(shape_.get());
                auto rectShape = rect->toShapeData();
                shapeCopy = std::make_unique<VectorRectangle>(rectShape.rect.x1, rectShape.rect.y1, 
                                                             rectShape.rect.x2, rectShape.rect.y2);
                break;
            }
            case SelectionShape::Circle: {
                auto* circle = static_cast<VectorCircle*>(shape_.get());
                auto circleShape = circle->toShapeData();
                shapeCopy = std::make_unique<VectorCircle>(circleShape.circle.centerX, circleShape.circle.centerY, 
                                                          circleShape.circle.radius);
                break;
            }
            case SelectionShape::Polygon: {
                auto* polygon = static_cast<VectorPolygon*>(shape_.get());
                auto polygonShape = polygon->toShapeData();
                shapeCopy = std::make_unique<VectorPolygon>(polygonShape.polygonVertices);
                break;
            }
        }
        
        selector_->InsertShapeInternal(std::move(shapeCopy), originalIndex_);
        executed_ = false;
    }
}

std::string DeleteShapeCommand::getDescription() const {
    return "删除图形";
}

// 已删除的命令类实现（MoveShapeCommand和EditShapeCommand已合并为StateChangeCommand）

void Selector::CompleteCurrentPolygon()
{
    if (currentPolygonVertices_.size() < 3) {
        qDebug() << "多边形顶点数不足3个，无法完成绘制";
        return;
    }
    
    // 将多边形添加到矢量图形列表（使用命令模式）
    std::vector<std::pair<double, double>> vertices;
    for (const auto& vertex : currentPolygonVertices_) {
        vertices.emplace_back(vertex.first, vertex.second);
    }
    
    auto newShape = std::make_unique<VectorPolygon>(vertices);
    auto addCommand = std::make_unique<AddShapeCommand>(this, std::move(newShape));
    ExecuteCommand(std::move(addCommand));
    
    qDebug() << "多边形已添加到画布，当前画布形状数：" << vectorShapes_.size();
    
    // 隐藏当前形状显示，显示矢量图形
    currentShapeActor_->SetVisibility(0);
    UpdateVectorShapeDisplay();
    
    // 清空顶点列表
    currentPolygonVertices_.clear();
    
    if (cursorCallback_) {
        cursorCallback_(Qt::ArrowCursor);
    }
    
    qDebug() << "多边形绘制完成";
}

void Selector::ConfirmSelection()
{
    if (!renderer_ || !pointCloudSelector_ || vectorShapes_.empty()) {
        qDebug() << "无法执行选择：缺少必要的数据或画布为空";
        return;
    }
    
    // 将矢量图形传递给PointCloudSelector进行选择
    std::vector<VectorShape*> shapePointers;
    for (const auto& vectorShape : vectorShapes_) {
        shapePointers.push_back(vectorShape.get());
    }
    
    // 使用PointCloudSelector进行点云选择
    std::vector<vtkIdType> selectedPoints = pointCloudSelector_->selectPointsByShapes(shapePointers);
    pointCloudSelector_->highlightSelectedPoints(selectedPoints);
    
    qDebug() << "选择完成，选中了" << selectedPoints.size() << "个点";
    
    // 选取完成后自动清空画布
    ClearCanvas();
}

void Selector::PerformCanvasBasedSelection()
{
    // 现在委托给PointCloudSelector
    if (!pointCloudSelector_ || vectorShapes_.empty()) return;
    
    std::vector<VectorShape*> shapePointers;
    for (const auto& vectorShape : vectorShapes_) {
        shapePointers.push_back(vectorShape.get());
    }
    
    std::vector<vtkIdType> selectedPoints = pointCloudSelector_->selectPointsByShapes(shapePointers);
    pointCloudSelector_->highlightSelectedPoints(selectedPoints);
}

void Selector::PerformCanvasBasedSelection(const std::vector<VectorShape*>& shapes)
{
    // 委托给PointCloudSelector
    if (!pointCloudSelector_ || shapes.empty()) return;
    
    std::vector<vtkIdType> selectedPoints = pointCloudSelector_->selectPointsByShapes(shapes);
    pointCloudSelector_->highlightSelectedPoints(selectedPoints);
    
    qDebug() << "选择完成，选中了" << selectedPoints.size() << "个点";
}

void Selector::ClearAllSelectedPoints()
{
    // 委托给PointCloudSelector
    if (pointCloudSelector_) {
        pointCloudSelector_->clearAllSelectedPoints();
    }
}

size_t Selector::GetSelectedPointCount() const
{
    // 委托给PointCloudSelector
    if (pointCloudSelector_) {
        return pointCloudSelector_->getSelectedPoints().size();
    }
    return 0;
}

// 已删除的方法实现 - 这些功能现在由PointCloudSelector类提供
// HighlightSelectedPoints, PerformOcclusionAwareSelection, FilterOccludedPoints, 
// IsPointOccluded, CalculateScreenDistance 等方法已移到PointCloudSelector中

//=============================================================================
// 状态模式实现 - 改进的状态管理
//=============================================================================

// DrawingState 实现
void DrawingState::handleMouseDown(Selector* selector, int x, int y) {
    selector->IsDrawing() = true;
    selector->StartX() = x;
    selector->StartY() = y;
    selector->CurrentX() = selector->StartX();
    selector->CurrentY() = selector->StartY();
    
    selector->CurrentShapeActor()->SetVisibility(1);
    selector->DrawCurrentShape();
    
    qDebug() << "DrawingState: 开始绘制形状";
}

void DrawingState::handleMouseMove(Selector* selector, int x, int y) {
    if (!selector->IsDrawing()) return;
    
    selector->CurrentX() = x;
    selector->CurrentY() = y;
    selector->DrawCurrentShape();
}

void DrawingState::handleMouseUp(Selector* selector) {
    if (!selector->IsDrawing()) return;
    
    selector->IsDrawing() = false;
    
    // 创建新形状并添加到画布
    std::unique_ptr<VectorShape> newShape;
    switch (selector->GetCurrentDrawingShape()) {
        case SelectionShape::Rectangle: {
            newShape = std::make_unique<VectorRectangle>(
                selector->StartX(), selector->StartY(), 
                selector->CurrentX(), selector->CurrentY());
            break;
        }
        case SelectionShape::Circle: {
            double centerX = (selector->StartX() + selector->CurrentX()) / 2.0;
            double centerY = (selector->StartY() + selector->CurrentY()) / 2.0;
            double radius = sqrt(pow(selector->CurrentX() - selector->StartX(), 2) + 
                               pow(selector->CurrentY() - selector->StartY(), 2)) / 2.0;
            newShape = std::make_unique<VectorCircle>(centerX, centerY, radius);
            break;
        }
        default:
            break;
    }
    
    if (newShape) {
        auto addCommand = std::make_unique<AddShapeCommand>(selector, std::move(newShape));
        selector->ExecuteCommand(std::move(addCommand));
        qDebug() << "DrawingState: 形状已添加到画布";
    }
    
    // 隐藏当前形状显示，显示矢量图形
    selector->CurrentShapeActor()->SetVisibility(0);
    selector->UpdateVectorShapeDisplay();
}

void DrawingState::handleRightClick(Selector* selector) {
    // 绘制状态下右键无特殊操作
}

void DrawingState::handleKeyPress(Selector* selector, const std::string& key, bool ctrlPressed) {
    if (ctrlPressed && key == "z") {
        selector->Undo();
    }
}

// EditingState 实现
void EditingState::handleMouseDown(Selector* selector, int x, int y) {
    // 检查是否点击了控制点
    if (selector->SelectedShape()) {
        int controlPointIndex = -1;
        if (selector->SelectedShape()->hitTestControlPoint(x, y, controlPointIndex)) {
            selector->CurrentOperation() = EditOperation::EditVertex;
            selector->SelectedControlPoint() = controlPointIndex;
            selector->IsDragging() = true;
            selector->LastMouseX() = x;
            selector->LastMouseY() = y;
            
            // 记录编辑前的状态
            selector->ShapeStateBeforeDrag() = selector->SelectedShape()->serialize();
            return;
        }
    }
    
    // 检查是否点击了图形
    VectorShape* clickedShape = selector->GetShapeAtPosition(x, y);
    if (clickedShape) {
        if (clickedShape != selector->SelectedShape()) {
            selector->SelectShape(clickedShape);
        }
        
        // 开始拖拽图形
        selector->CurrentOperation() = EditOperation::Move;
        selector->IsDragging() = true;
        selector->LastMouseX() = x;
        selector->LastMouseY() = y;
        
        // 记录拖拽前的状态
        selector->ShapeStateBeforeDrag() = selector->SelectedShape()->serialize();
        selector->SelectedShape()->startDrag(x, y);
    } else {
        // 点击空白区域，取消选择
        selector->DeselectAllShapes();
    }
}

void EditingState::handleMouseMove(Selector* selector, int x, int y) {
    if (selector->IsDragging() && selector->SelectedShape()) {
        if (selector->CurrentOperation() == EditOperation::Move) {
            selector->SelectedShape()->updateDrag(x, y);
            selector->UpdateVectorShapeDisplay();
        } else if (selector->CurrentOperation() == EditOperation::EditVertex && 
                   selector->SelectedControlPoint() >= 0) {
            selector->SelectedShape()->moveControlPoint(selector->SelectedControlPoint(), x, y);
            selector->UpdateVectorShapeDisplay();
        }
        
        selector->LastMouseX() = x;
        selector->LastMouseY() = y;
    }
}

void EditingState::handleMouseUp(Selector* selector) {
    if (selector->IsDragging()) {
        if (selector->SelectedShape()) {
            selector->SelectedShape()->endDrag();
            
            // 记录拖拽后的状态并创建命令
            std::string stateAfterDrag = selector->SelectedShape()->serialize();
            if (stateAfterDrag != selector->ShapeStateBeforeDrag()) {
                std::string description = (selector->CurrentOperation() == EditOperation::Move) ? "移动图形" : "编辑图形";
                auto command = std::make_unique<StateChangeCommand>(
                    selector->SelectedShape(), 
                    selector->ShapeStateBeforeDrag(), 
                    stateAfterDrag, 
                    description);
                
                selector->ExecuteCommand(std::move(command));
            }
            
            selector->UpdateVectorShapeDisplay();
        }
        
        selector->IsDragging() = false;
        selector->CurrentOperation() = EditOperation::None;
        selector->SelectedControlPoint() = -1;
        selector->ShapeStateBeforeDrag().clear();
    }
}

void EditingState::handleRightClick(Selector* selector) {
    // 编辑状态下右键无特殊操作
}

void EditingState::handleKeyPress(Selector* selector, const std::string& key, bool ctrlPressed) {
    if (ctrlPressed && key == "z") {
        selector->Undo();
    } else if (key == "Delete") {
        selector->DeleteSelectedShape();
    }
}

// PolygonDrawingState 实现
void PolygonDrawingState::handleMouseDown(Selector* selector, int x, int y) {
    selector->CurrentX() = x;
    selector->CurrentY() = y;
    selector->AddPolygonVertex(x, y);
}

void PolygonDrawingState::handleMouseMove(Selector* selector, int x, int y) {
    selector->CurrentX() = x;
    selector->CurrentY() = y;
    selector->DrawCurrentShape();
}

void PolygonDrawingState::handleMouseUp(Selector* selector) {
    // 多边形绘制时鼠标抬起无特殊操作
}

void PolygonDrawingState::handleRightClick(Selector* selector) {
    selector->CompleteCurrentPolygon();
}

void PolygonDrawingState::handleKeyPress(Selector* selector, const std::string& key, bool ctrlPressed) {
    if (ctrlPressed && key == "z") {
        selector->Undo();
    } else if (key == "BackSpace") {
        selector->UndoLastVertex();
    }
}