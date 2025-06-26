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

Shape VectorRectangle::toShape() const {
    Shape shape(SelectionShape::Rectangle);
    shape.rect.x1 = x1_;
    shape.rect.y1 = y1_;
    shape.rect.x2 = x2_;
    shape.rect.y2 = y2_;
    return shape;
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

Shape VectorCircle::toShape() const {
    Shape shape(SelectionShape::Circle);
    shape.circle.centerX = centerX_;
    shape.circle.centerY = centerY_;
    shape.circle.radius = radius_;
    return shape;
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

Shape VectorPolygon::toShape() const {
    Shape shape(SelectionShape::Polygon);
    shape.polygon.vertices = vertices_;
    return shape;
}

//=============================================================================
// Selector 实现
//=============================================================================

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
    , selectedShape(nullptr)
    , currentOperation(EditOperation::None)
    , selectedControlPoint(-1)
    , isDragging(false)
    , lastMouseX(0)
    , lastMouseY(0)
{
    // 初始化矢量图形显示
    vectorShapePoints = vtkSmartPointer<vtkPoints>::New();
    vectorShapePolyData = vtkSmartPointer<vtkPolyData>::New();
    vectorShapePolyData->SetPoints(vectorShapePoints);
    vectorShapeMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    vectorShapeMapper->SetInputData(vectorShapePolyData);
    vectorShapeActor = vtkSmartPointer<vtkActor2D>::New();
    vectorShapeActor->SetMapper(vectorShapeMapper);
    
    // 初始化控制点显示
    controlPoints = vtkSmartPointer<vtkPoints>::New();
    controlPolyData = vtkSmartPointer<vtkPolyData>::New();
    controlPolyData->SetPoints(controlPoints);
    controlMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    controlMapper->SetInputData(controlPolyData);
    controlActor = vtkSmartPointer<vtkActor2D>::New();
    controlActor->SetMapper(controlMapper);
    controlActor->GetProperty()->SetColor(1.0, 1.0, 0.0); // 黄色控制点
    controlActor->GetProperty()->SetPointSize(8.0);
    
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
    
    // 设置2D坐标系统为屏幕坐标
    vtkSmartPointer<vtkCoordinate> coord = vtkSmartPointer<vtkCoordinate>::New();
    coord->SetCoordinateSystem(0); // VTK_DISPLAY = 0，屏幕坐标
    vectorShapeMapper->SetTransformCoordinate(coord);
    controlMapper->SetTransformCoordinate(coord);
    currentShapeMapper->SetTransformCoordinate(coord);
}

Selector::~Selector()
{
}

void Selector::SetRenderer(vtkRenderer* ren)
{
    renderer = ren;
    if (renderer) {
        renderer->AddActor2D(vectorShapeActor);
        renderer->AddActor2D(controlActor);
        renderer->AddActor2D(currentShapeActor);
        vectorShapeActor->SetVisibility(0);
        controlActor->SetVisibility(0);
        currentShapeActor->SetVisibility(0);
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
    vectorShapes.clear();
    DeselectAllShapes();
    vectorShapeActor->SetVisibility(0);
    controlActor->SetVisibility(0);
    if (renderer) {
        renderer->GetRenderWindow()->Render();
    }
    qDebug() << "画布已清空";
}

void Selector::ClearCurrentDrawing()
{
    isDrawing = false;
    isDrawingPolygon = false;
    currentPolygonVertices.clear();
    currentShapeActor->SetVisibility(0);
    
    if (renderer) {
        renderer->GetRenderWindow()->Render();
    }
    
    qDebug() << "已清除当前绘制状态";
}

void Selector::UpdateVectorShapeDisplay()
{
    if (!renderer || vectorShapes.empty()) {
        vectorShapeActor->SetVisibility(0);
        controlActor->SetVisibility(0);
        if (renderer) {
            renderer->GetRenderWindow()->Render();
        }
        return;
    }
    
    // 计算所有形状所需的总点数
    size_t totalPoints = 0;
    for (const auto& shape : vectorShapes) {
        switch (shape->getType()) {
            case SelectionShape::Rectangle:
                totalPoints += 4;
                break;
            case SelectionShape::Circle:
                totalPoints += 64;
                break;
            case SelectionShape::Polygon: {
                auto* polygon = static_cast<VectorPolygon*>(shape.get());
                totalPoints += polygon->toShape().polygon.vertices.size();
                break;
            }
        }
    }
    
    if (totalPoints == 0) {
        vectorShapeActor->SetVisibility(0);
        controlActor->SetVisibility(0);
        if (renderer) {
            renderer->GetRenderWindow()->Render();
        }
        return;
    }
    
    // 绘制所有矢量形状
    vectorShapePoints->SetNumberOfPoints(totalPoints);
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    
    size_t pointIndex = 0;
    for (const auto& shape : vectorShapes) {
        // 每个形状单独绘制，以便设置不同的颜色
        vtkSmartPointer<vtkPoints> shapePoints = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPolyData> shapePolyData = vtkSmartPointer<vtkPolyData>::New();
        shapePolyData->SetPoints(shapePoints);
        
        shape->draw(renderer, vectorShapeActor, vectorShapeMapper, shapePolyData, shapePoints);
        
        // 复制点到主要的点集
        for (vtkIdType i = 0; i < shapePoints->GetNumberOfPoints(); ++i) {
            double point[3];
            shapePoints->GetPoint(i, point);
            vectorShapePoints->SetPoint(pointIndex + i, point);
        }
        
        // 复制线到主要的线集
        vtkCellArray* shapeLines = shapePolyData->GetLines();
        if (shapeLines) {
            shapeLines->InitTraversal();
            vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
            while (shapeLines->GetNextCell(idList)) {
                vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
                for (vtkIdType i = 0; i < idList->GetNumberOfIds(); ++i) {
                    line->GetPointIds()->SetId(i, pointIndex + idList->GetId(i));
                }
                lines->InsertNextCell(line);
            }
        }
        
        pointIndex += shapePoints->GetNumberOfPoints();
    }
    
    vectorShapePolyData->SetLines(lines);
    vectorShapePoints->Modified();
    vectorShapePolyData->Modified();
    vectorShapeActor->SetVisibility(1);
    
    // 绘制控制点
    DrawControlPoints();
    
    if (renderer) {
        renderer->GetRenderWindow()->Render();
    }
}

void Selector::DrawControlPoints()
{
    if (!selectedShape) {
        controlActor->SetVisibility(0);
        return;
    }
    
    std::vector<std::pair<double, double>> controlPts;
    
    if (selectedShape->getType() == SelectionShape::Rectangle) {
        auto* rect = static_cast<VectorRectangle*>(selectedShape);
        auto rectShape = rect->toShape();
        controlPts = {
            {rectShape.rect.x1, rectShape.rect.y1}, {rectShape.rect.x2, rectShape.rect.y1}, 
            {rectShape.rect.x2, rectShape.rect.y2}, {rectShape.rect.x1, rectShape.rect.y2}, // 四个角
            {(rectShape.rect.x1 + rectShape.rect.x2) / 2, rectShape.rect.y1}, 
            {rectShape.rect.x2, (rectShape.rect.y1 + rectShape.rect.y2) / 2}, 
            {(rectShape.rect.x1 + rectShape.rect.x2) / 2, rectShape.rect.y2}, 
            {rectShape.rect.x1, (rectShape.rect.y1 + rectShape.rect.y2) / 2}  // 四个边中点
        };
    } else if (selectedShape->getType() == SelectionShape::Circle) {
        auto* circle = static_cast<VectorCircle*>(selectedShape);
        auto circleShape = circle->toShape();
        controlPts = {
            {circleShape.circle.centerX + circleShape.circle.radius, circleShape.circle.centerY}, // 右
            {circleShape.circle.centerX, circleShape.circle.centerY - circleShape.circle.radius}, // 上
            {circleShape.circle.centerX - circleShape.circle.radius, circleShape.circle.centerY}, // 左
            {circleShape.circle.centerX, circleShape.circle.centerY + circleShape.circle.radius}  // 下
        };
    } else if (selectedShape->getType() == SelectionShape::Polygon) {
        auto* polygon = static_cast<VectorPolygon*>(selectedShape);
        auto polygonShape = polygon->toShape();
        controlPts = polygonShape.polygon.vertices;
    }
    
    if (controlPts.empty()) {
        controlActor->SetVisibility(0);
        return;
    }
    
    controlPoints->SetNumberOfPoints(controlPts.size());
    for (size_t i = 0; i < controlPts.size(); ++i) {
        controlPoints->SetPoint(i, controlPts[i].first, controlPts[i].second, 0);
    }
    
    // 创建顶点
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    for (size_t i = 0; i < controlPts.size(); ++i) {
        vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
        vertex->GetPointIds()->SetId(0, i);
        vertices->InsertNextCell(vertex);
    }
    controlPolyData->SetVerts(vertices);
    
    controlPoints->Modified();
    controlPolyData->Modified();
    controlActor->SetVisibility(1);
}

VectorShape* Selector::GetShapeAtPosition(double x, double y)
{
    for (auto& shape : vectorShapes) {
        if (shape->hitTest(x, y)) {
            return shape.get();
        }
    }
    return nullptr;
}

void Selector::SelectShape(VectorShape* shape)
{
    DeselectAllShapes();
    if (shape) {
        selectedShape = shape;
        shape->setSelected(true);
        UpdateVectorShapeDisplay();
        
        if (cursorCallback) {
            cursorCallback(Qt::SizeAllCursor);
        }
    }
}

void Selector::DeselectAllShapes()
{
    selectedShape = nullptr;
    for (auto& shape : vectorShapes) {
        shape->setSelected(false);
    }
    controlActor->SetVisibility(0);
    
    if (cursorCallback) {
        cursorCallback(Qt::ArrowCursor);
    }
}

void Selector::OnLeftButtonDown()
{
    if (!drawingModeEnabled) {
        // 编辑模式：处理图形选择和编辑
        int x, y;
        this->GetInteractor()->GetEventPosition(x, y);
        
        // 检查是否点击了控制点
        if (selectedShape) {
            int controlPointIndex = -1;
            if (selectedShape->hitTestControlPoint(x, y, controlPointIndex)) {
                currentOperation = EditOperation::EditVertex;
                selectedControlPoint = controlPointIndex;
                isDragging = true;
                lastMouseX = x;
                lastMouseY = y;
                
                if (cursorCallback) {
                    cursorCallback(Qt::PointingHandCursor);
                }
                return;
            }
        }
        
        // 检查是否点击了图形
        VectorShape* clickedShape = GetShapeAtPosition(x, y);
        if (clickedShape) {
            if (clickedShape != selectedShape) {
                SelectShape(clickedShape);
            }
            
            // 开始拖拽图形
            currentOperation = EditOperation::Move;
            isDragging = true;
            lastMouseX = x;
            lastMouseY = y;
            selectedShape->startDrag(x, y);
            
            if (cursorCallback) {
                cursorCallback(Qt::SizeAllCursor);
            }
        } else {
            // 点击空白区域，取消选择
            DeselectAllShapes();
            
            if (!viewLocked) {
                vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
            }
        }
        return;
    }
    
    // 绘制模式：原有逻辑
    int x, y;
    this->GetInteractor()->GetEventPosition(x, y);
    
    if (currentDrawingShape == SelectionShape::Polygon) {
        if (!isDrawingPolygon) {
            isDrawingPolygon = true;
            currentPolygonVertices.clear();
        }
        
        currentX = x;
        currentY = y;
        AddPolygonVertex(x, y);
        
        if (cursorCallback) {
            cursorCallback(Qt::CrossCursor);
        }
    } else {
        isDrawing = true;
        startX = x;
        startY = y;
        currentX = startX;
        currentY = startY;
        
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
        // 编辑模式：结束拖拽操作
        if (isDragging) {
            if (selectedShape) {
                selectedShape->endDrag();
                UpdateVectorShapeDisplay();
            }
            
            isDragging = false;
            currentOperation = EditOperation::None;
            selectedControlPoint = -1;
            
            if (cursorCallback) {
                if (selectedShape) {
                    cursorCallback(Qt::SizeAllCursor);
                } else {
                    cursorCallback(Qt::ArrowCursor);
                }
            }
        } else if (!viewLocked) {
            vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
        }
        return;
    }
    
    // 绘制模式：原有逻辑
    if (currentDrawingShape == SelectionShape::Polygon) {
        return;
    }
    
    if (!isDrawing) {
        if (!viewLocked) {
            vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
        }
        return;
    }
    
    isDrawing = false;
    
    // 将当前形状添加到矢量图形列表
    switch (currentDrawingShape) {
        case SelectionShape::Rectangle: {
            vectorShapes.push_back(std::make_unique<VectorRectangle>(startX, startY, currentX, currentY));
            break;
        }
        case SelectionShape::Circle: {
            double centerX = (startX + currentX) / 2.0;
            double centerY = (startY + currentY) / 2.0;
            double radius = sqrt(pow(currentX - startX, 2) + pow(currentY - startY, 2)) / 2.0;
            vectorShapes.push_back(std::make_unique<VectorCircle>(centerX, centerY, radius));
            break;
        }
        default:
            break;
    }
    
    qDebug() << "形状已添加到画布，当前画布形状数：" << vectorShapes.size();
    
    // 隐藏当前形状显示，显示矢量图形
    currentShapeActor->SetVisibility(0);
    UpdateVectorShapeDisplay();
    
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
    
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();
    
    if (currentDrawingShape == SelectionShape::Polygon && isDrawingPolygon) {
        if (key == "BackSpace") {
            UndoLastVertex();
            return;
        }
    }
    
    if (!viewLocked) {
        vtkInteractorStyleTrackballCamera::OnKeyPress();
    }
}

void Selector::OnMouseMove()
{
    int x, y;
    this->GetInteractor()->GetEventPosition(x, y);
    
    if (!drawingModeEnabled) {
        // 编辑模式：处理拖拽操作
        if (isDragging && selectedShape) {
            // 检查是否按下了Shift键
            bool shiftPressed = (this->GetInteractor()->GetShiftKey() != 0);
            
            if (currentOperation == EditOperation::Move) {
                selectedShape->updateDrag(x, y, shiftPressed);
                UpdateVectorShapeDisplay();
            } else if (currentOperation == EditOperation::EditVertex && selectedControlPoint >= 0) {
                selectedShape->moveControlPoint(selectedControlPoint, x, y);
                UpdateVectorShapeDisplay();
            }
            
            lastMouseX = x;
            lastMouseY = y;
        } else if (!viewLocked) {
            vtkInteractorStyleTrackballCamera::OnMouseMove();
        }
        return;
    }
    
    // 绘制模式：原有逻辑
    if (!isDrawing && !isDrawingPolygon) {
        if (!viewLocked) {
            vtkInteractorStyleTrackballCamera::OnMouseMove();
        }
        return;
    }
    
    currentX = x;
    currentY = y;
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
        
        // 连接最后一个点到第一个点形成闭合图形
        // 如果正在绘制且顶点数>=3，或者已完成且顶点数>2，都显示闭合线
        if ((isDrawingPolygon && vertices.size() >= 3) || (!isDrawingPolygon && vertices.size() > 2)) {
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
    
    // 将多边形添加到矢量图形列表
    std::vector<std::pair<double, double>> vertices;
    for (const auto& vertex : currentPolygonVertices) {
        vertices.emplace_back(vertex.first, vertex.second);
    }
    vectorShapes.push_back(std::make_unique<VectorPolygon>(vertices));
    
    qDebug() << "多边形已添加到画布，当前画布形状数：" << vectorShapes.size();
    
    // 隐藏当前形状显示，显示矢量图形
    currentShapeActor->SetVisibility(0);
    UpdateVectorShapeDisplay();
    
    // 清空顶点列表
    currentPolygonVertices.clear();
    
    if (cursorCallback) {
        cursorCallback(Qt::ArrowCursor);
    }
    
    qDebug() << "多边形绘制完成";
}

void Selector::ConfirmSelection()
{
    if (!renderer || !originalPointData || vectorShapes.empty()) {
        qDebug() << "无法执行选择：缺少必要的数据或画布为空";
        return;
    }
    
    // 将矢量图形转换为原有的Shape格式进行选取
    std::vector<Shape> shapes;
    for (const auto& vectorShape : vectorShapes) {
        shapes.push_back(vectorShape->toShape());
    }
    
    // 基于转换后的形状进行点云选择
    PerformCanvasBasedSelection(shapes);
    
    // 选取完成后自动清空画布
    ClearCanvas();
}

void Selector::PerformCanvasBasedSelection()
{
    // 将矢量图形转换为原有的Shape格式进行选取
    std::vector<Shape> shapes;
    for (const auto& vectorShape : vectorShapes) {
        shapes.push_back(vectorShape->toShape());
    }
    
    // 调用带参数的版本
    PerformCanvasBasedSelection(shapes);
}

void Selector::PerformCanvasBasedSelection(const std::vector<Shape>& shapes)
{
    if (!renderer || !originalPointData || shapes.empty()) return;
    
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
        
        // 使用转换后的形状进行判断
        if (IsPointInShapes(screenPoint[0], screenPoint[1], shapes)) {
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

bool Selector::IsPointInShapes(double screenX, double screenY, const std::vector<Shape>& shapes)
{
    for (const auto& shape : shapes) {
        if (shape.containsPoint(screenX, screenY)) {
            return true;
        }
    }
    return false;
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