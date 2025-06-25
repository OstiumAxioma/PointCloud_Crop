#ifndef SELECTOR_H
#define SELECTOR_H

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellPicker.h>
#include <vtkPropPicker.h>
#include <vtkExtractPoints.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkActor2D.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkCoordinate.h>
#include <vtkProperty2D.h>
#include <vtkLookupTable.h>
#include <vtkDataArray.h>
#include <QObject>
#include <functional>
#include <vector>
#include <map>

// 选择框形状枚举
enum class SelectionShape {
    Rectangle,
    Circle,
    Polygon
};

class Selector : public vtkInteractorStyleTrackballCamera
{
public:
    static Selector* New();
    vtkTypeMacro(Selector, vtkInteractorStyleTrackballCamera);

    // 设置渲染器和点云数据
    void SetRenderer(vtkRenderer* renderer);
    void SetPointCloudData(vtkPolyData* pointData);
    
    // 启用/禁用矩形选择模式
    void EnableRectangleSelection(bool enable);
    bool IsRectangleSelectionEnabled() const { return rectangleSelectionEnabled; }

    // 启用/禁用遮挡检测
    void EnableOcclusionDetection(bool enable) { occlusionDetectionEnabled = enable; }
    bool IsOcclusionDetectionEnabled() const { return occlusionDetectionEnabled; }

    // 设置选择框形状
    void SetSelectionShape(SelectionShape shape) { selectionShape = shape; }
    SelectionShape GetSelectionShape() const { return selectionShape; }

    // 清除所有选中的点
    void ClearAllSelectedPoints();
    
    // 获取当前选中的点数量
    size_t GetSelectedPointCount() const { return selectedPointIds.size(); }
    
    // 设置光标回调函数
    void SetCursorCallback(std::function<void(Qt::CursorShape)> callback) {
        cursorCallback = callback;
    }

protected:
    Selector();
    ~Selector();

    // 鼠标事件处理
    virtual void OnLeftButtonDown() override;
    virtual void OnLeftButtonUp() override;
    virtual void OnRightButtonDown() override;
    virtual void OnMouseMove() override;

private:
    // 绘制选择矩形
    void DrawSelectionRectangle();
    void ClearSelectionRectangle();
    
    // 绘制选择圆形
    void DrawSelectionCircle();
    void ClearSelectionCircle();
    
    // 绘制选择多边形
    void DrawSelectionPolygon();
    void ClearSelectionPolygon();
    void AddPolygonVertex(int x, int y);
    void CompletePolygonSelection();
    bool IsPointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon);
    
    // 绘制选择框（通用方法）
    void DrawSelectionShape();
    void ClearSelectionShape();
    
    // 执行点云选择
    void PerformPointSelection();
    
    // 高亮选中的点
    void HighlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds);

    // 遮挡检测相关方法
    void PerformOcclusionAwareSelection();
    std::vector<vtkIdType> FilterOccludedPoints(const std::vector<vtkIdType>& candidatePoints,
                                                    const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                                                    const std::map<vtkIdType, double>& distancesToCamera);
    double CalculateScreenDistance(double x1, double y1, double x2, double y2);
    bool IsPointOccluded(vtkIdType pointId, const std::vector<vtkIdType>& frontPoints, 
                        const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                        double occlusionThreshold = 5.0);
    
    // 判断点是否在选择区域内
    bool IsPointInSelectionArea(double screenX, double screenY);

    // 成员变量
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkPolyData> originalPointData;
    vtkSmartPointer<vtkActor> selectionActor;
    vtkSmartPointer<vtkPolyDataMapper> selectionMapper;
    vtkSmartPointer<vtkPolyData> selectionPolyData;
    
    bool rectangleSelectionEnabled;
    bool isSelecting;
    int startX, startY;
    int currentX, currentY;
    
    // 多边形相关变量
    bool isDrawingPolygon;
    std::vector<std::pair<int, int>> polygonVertices;
    
    // 选择框的四个角点
    vtkSmartPointer<vtkPoints> rectanglePoints;
    vtkSmartPointer<vtkPolyData> rectanglePolyData;
    vtkSmartPointer<vtkActor2D> rectangleActor;
    vtkSmartPointer<vtkPolyDataMapper2D> rectangleMapper;

    std::function<void(Qt::CursorShape)> cursorCallback;
    
    // 选中的点ID列表
    std::vector<vtkIdType> selectedPointIds;
    
    // 原始颜色备份
    std::vector<unsigned char> originalColorBackup;

    // 遮挡检测标志
    bool occlusionDetectionEnabled;

    SelectionShape selectionShape;
};

#endif // SELECTOR_H 