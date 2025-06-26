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
#include <memory>
#include <stack>

// 选择框形状枚举
enum class SelectionShape {
    Rectangle,
    Circle,
    Polygon
};

// 编辑操作类型枚举
enum class EditOperation {
    None,
    Move,
    Resize,
    EditVertex
};

// 前向声明
class VectorShape;
class Selector;

// 命令基类
class Command {
public:
    virtual ~Command() = default;
    virtual void execute() = 0;
    virtual void undo() = 0;
    virtual std::string getDescription() const = 0;
};

// 添加图形命令
class AddShapeCommand : public Command {
public:
    AddShapeCommand(Selector* selector, std::unique_ptr<VectorShape> shape);
    void execute() override;
    void undo() override;
    std::string getDescription() const override;

private:
    Selector* selector_;
    std::unique_ptr<VectorShape> shape_;
    VectorShape* shapePtr_; // 保存原始指针，用于undo时查找
    bool executed_;
};

// 删除图形命令
class DeleteShapeCommand : public Command {
public:
    DeleteShapeCommand(Selector* selector, VectorShape* shape);
    void execute() override;
    void undo() override;
    std::string getDescription() const override;

private:
    Selector* selector_;
    std::unique_ptr<VectorShape> shape_;
    VectorShape* originalShapePtr_; // 保存原始形状指针用于删除
    size_t originalIndex_;
    bool executed_;
};

// 移动图形命令
class MoveShapeCommand : public Command {
public:
    MoveShapeCommand(VectorShape* shape, double fromX, double fromY, double toX, double toY);
    void execute() override;
    void undo() override;
    std::string getDescription() const override;

private:
    VectorShape* shape_;
    double fromX_, fromY_, toX_, toY_;
};

// 编辑图形命令（包括顶点移动、缩放等）
class EditShapeCommand : public Command {
public:
    EditShapeCommand(VectorShape* shape, const std::string& beforeState, const std::string& afterState);
    void execute() override;
    void undo() override;
    std::string getDescription() const override;

private:
    VectorShape* shape_;
    std::string beforeState_;
    std::string afterState_;
};

// 形状数据结构（保持原有结构用于最终选取）
struct Shape {
    SelectionShape type;
    
    // 矩形参数：左上角和右下角
    struct Rectangle {
        double x1, y1, x2, y2;
    };
    
    // 圆形参数：中心点和半径
    struct Circle {
        double centerX, centerY, radius;
    };
    
    // 多边形参数：顶点列表
    struct Polygon {
        std::vector<std::pair<double, double>> vertices;
    };
    
    union {
        Rectangle rect;
        Circle circle;
        Polygon polygon;
    };
    
    Shape(SelectionShape t) : type(t) {
        if (type == SelectionShape::Polygon) {
            new(&polygon) Polygon();
        }
    }
    
    ~Shape() {
        if (type == SelectionShape::Polygon) {
            polygon.~Polygon();
        }
    }
    
    // 拷贝构造函数
    Shape(const Shape& other) : type(other.type) {
        switch (type) {
            case SelectionShape::Rectangle:
                rect = other.rect;
                break;
            case SelectionShape::Circle:
                circle = other.circle;
                break;
            case SelectionShape::Polygon:
                new(&polygon) Polygon(other.polygon);
                break;
        }
    }
    
    // 赋值运算符
    Shape& operator=(const Shape& other) {
        if (this != &other) {
            if (type == SelectionShape::Polygon) {
                polygon.~Polygon();
            }
            type = other.type;
            switch (type) {
                case SelectionShape::Rectangle:
                    rect = other.rect;
                    break;
                case SelectionShape::Circle:
                    circle = other.circle;
                    break;
                case SelectionShape::Polygon:
                    new(&polygon) Polygon(other.polygon);
                    break;
            }
        }
        return *this;
    }
    
    // 判断点是否在形状内
    bool containsPoint(double x, double y) const;
};

// 矢量图形基类
class VectorShape {
public:
    virtual ~VectorShape() = default;
    
    // 绘制图形
    virtual void draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, vtkPolyData* polyData, vtkPoints* points) = 0;
    
    // 碰撞检测
    virtual bool hitTest(double x, double y, double tolerance = 8.0) const = 0;
    
    // 控制点碰撞检测
    virtual bool hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance = 8.0) const = 0;
    
    // 拖拽操作
    virtual void startDrag(double x, double y) = 0;
    virtual void updateDrag(double x, double y, bool shiftPressed = false) = 0;
    virtual void endDrag() = 0;
    
    // 移动控制点
    virtual void moveControlPoint(int index, double x, double y) = 0;
    
    // 转换为原有的Shape格式（用于选取）
    virtual Shape toShape() const = 0;
    
    // 选择状态
    virtual void setSelected(bool selected) { isSelected_ = selected; }
    virtual bool isSelected() const { return isSelected_; }
    
    // 获取形状类型
    virtual SelectionShape getType() const = 0;
    
    // 序列化状态（用于撤销功能）
    virtual std::string serialize() const = 0;
    virtual void deserialize(const std::string& data) = 0;
    
protected:
    bool isSelected_ = false;
};

// 矢量矩形
class VectorRectangle : public VectorShape {
public:
    VectorRectangle(double x1, double y1, double x2, double y2);
    
    void draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, vtkPolyData* polyData, vtkPoints* points) override;
    bool hitTest(double x, double y, double tolerance = 8.0) const override;
    bool hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance = 8.0) const override;
    void startDrag(double x, double y) override;
    void updateDrag(double x, double y, bool shiftPressed = false) override;
    void endDrag() override;
    void moveControlPoint(int index, double x, double y) override;
    Shape toShape() const override;
    SelectionShape getType() const override { return SelectionShape::Rectangle; }
    
    std::string serialize() const override;
    void deserialize(const std::string& data) override;
    
private:
    double x1_, y1_, x2_, y2_; // 矩形坐标
    double dragStartX_, dragStartY_; // 拖拽起始点
    double originalX1_, originalY1_, originalX2_, originalY2_; // 拖拽前的原始坐标
    
    void normalizeCoordinates(); // 确保坐标顺序正确
    std::vector<std::pair<double, double>> getControlPoints() const; // 获取控制点位置
};

// 矢量圆形
class VectorCircle : public VectorShape {
public:
    VectorCircle(double centerX, double centerY, double radius);
    
    void draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, vtkPolyData* polyData, vtkPoints* points) override;
    bool hitTest(double x, double y, double tolerance = 8.0) const override;
    bool hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance = 8.0) const override;
    void startDrag(double x, double y) override;
    void updateDrag(double x, double y, bool shiftPressed = false) override;
    void endDrag() override;
    void moveControlPoint(int index, double x, double y) override;
    Shape toShape() const override;
    SelectionShape getType() const override { return SelectionShape::Circle; }
    
    std::string serialize() const override;
    void deserialize(const std::string& data) override;
    
private:
    double centerX_, centerY_, radius_; // 圆心和半径
    double dragStartX_, dragStartY_; // 拖拽起始点
    double originalCenterX_, originalCenterY_, originalRadius_; // 拖拽前的原始值
    
    std::vector<std::pair<double, double>> getControlPoints() const; // 获取控制点位置
};

// 矢量多边形
class VectorPolygon : public VectorShape {
public:
    VectorPolygon(const std::vector<std::pair<double, double>>& vertices);
    
    void draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, vtkPolyData* polyData, vtkPoints* points) override;
    bool hitTest(double x, double y, double tolerance = 8.0) const override;
    bool hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance = 8.0) const override;
    void startDrag(double x, double y) override;
    void updateDrag(double x, double y, bool shiftPressed = false) override;
    void endDrag() override;
    void moveControlPoint(int index, double x, double y) override;
    Shape toShape() const override;
    SelectionShape getType() const override { return SelectionShape::Polygon; }
    
    std::string serialize() const override;
    void deserialize(const std::string& data) override;
    
private:
    std::vector<std::pair<double, double>> vertices_; // 顶点列表
    double dragStartX_, dragStartY_; // 拖拽起始点
    std::vector<std::pair<double, double>> originalVertices_; // 拖拽前的原始顶点
    
    bool isPointInPolygon(double x, double y) const; // 判断点是否在多边形内
};

class Selector : public vtkInteractorStyleTrackballCamera
{
public:
    static Selector* New();
    vtkTypeMacro(Selector, vtkInteractorStyleTrackballCamera);

    // 设置渲染器和点云数据
    void SetRenderer(vtkRenderer* renderer);
    void SetPointCloudData(vtkPolyData* pointData);
    
    // 启用/禁用绘制模式
    void EnableDrawingMode(bool enable);
    bool IsDrawingModeEnabled() const { return drawingModeEnabled; }

    // 启用/禁用遮挡检测
    void EnableOcclusionDetection(bool enable) { occlusionDetectionEnabled = enable; }
    bool IsOcclusionDetectionEnabled() const { return occlusionDetectionEnabled; }
    
    // 启用/禁用视图锁定（禁用相机操作）
    void EnableViewLock(bool enable) { viewLocked = enable; }
    bool IsViewLocked() const { return viewLocked; }

    // 设置当前绘制的形状类型
    void SetCurrentDrawingShape(SelectionShape shape) { currentDrawingShape = shape; }
    SelectionShape GetCurrentDrawingShape() const { return currentDrawingShape; }

    // 画布操作
    void ClearCanvas(); // 清空所有已绘制的形状
    void ConfirmSelection(); // 根据画布上的形状进行选取
    size_t GetCanvasShapeCount() const { return vectorShapes.size(); }
    
    // 清除所有选中的点
    void ClearAllSelectedPoints();
    
    // 清除当前正在绘制的形状
    void ClearCurrentDrawing();
    
    // 获取当前选中的点数量
    size_t GetSelectedPointCount() const { return selectedPointIds.size(); }
    
    // 设置光标回调函数
    void SetCursorCallback(std::function<void(Qt::CursorShape)> callback) {
        cursorCallback = callback;
    }
    
    // 撤销系统
    void ExecuteCommand(std::unique_ptr<Command> command);
    void Undo();
    bool CanUndo() const { return !commandHistory.empty(); }
    
    // 删除选中的图形
    void DeleteSelectedShape();
    
    // 内部方法（供命令类调用）
    void AddShapeInternal(std::unique_ptr<VectorShape> shape);
    std::unique_ptr<VectorShape> RemoveShapeInternal(VectorShape* shape);
    void InsertShapeInternal(std::unique_ptr<VectorShape> shape, size_t index);
    size_t GetShapeIndex(VectorShape* shape) const;

protected:
    Selector();
    ~Selector();

    // 鼠标事件处理
    virtual void OnLeftButtonDown() override;
    virtual void OnLeftButtonUp() override;
    virtual void OnRightButtonDown() override;
    virtual void OnMouseMove() override;
    
    // 键盘事件处理
    virtual void OnKeyPress() override;

private:
    // 矢量图形编辑相关方法
    void UpdateVectorShapeDisplay();
    void DrawControlPoints();
    VectorShape* GetShapeAtPosition(double x, double y);
    void SelectShape(VectorShape* shape);
    void DeselectAllShapes();
    
    // 绘制当前正在编辑的形状（绘制模式）
    void DrawCurrentShape();
    void ClearCurrentShapeDisplay();
    
    // 绘制矩形（绘制模式）
    void DrawRectangle(double x1, double y1, double x2, double y2);
    
    // 绘制圆形（绘制模式）
    void DrawCircle(double centerX, double centerY, double radius);
    
    // 绘制多边形（绘制模式）
    void DrawPolygon(const std::vector<std::pair<double, double>>& vertices, bool addTemporaryVertex = true);
    void AddPolygonVertex(int x, int y);
    void UndoLastVertex();
    void CompleteCurrentPolygon();
    
    // 执行点云选择（基于画布上的所有形状）
    void PerformCanvasBasedSelection();
    void PerformCanvasBasedSelection(const std::vector<Shape>& shapes);
    
    // 高亮选中的点
    void HighlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds);

    // 遮挡检测相关方法
    void PerformOcclusionAwareSelection(const std::vector<vtkIdType>& candidatePoints);
    std::vector<vtkIdType> FilterOccludedPoints(const std::vector<vtkIdType>& candidatePoints,
                                                    const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                                                    const std::map<vtkIdType, double>& distancesToCamera);
    double CalculateScreenDistance(double x1, double y1, double x2, double y2);
    bool IsPointOccluded(vtkIdType pointId, const std::vector<vtkIdType>& frontPoints, 
                        const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                        double occlusionThreshold = 5.0);
    
    // 判断点是否在形状内（用于选取）
    bool IsPointInShapes(double screenX, double screenY, const std::vector<Shape>& shapes);

    // 成员变量
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkPolyData> originalPointData;
    
    // 矢量图形显示相关（每个图形独立的actor）
    std::vector<vtkSmartPointer<vtkActor2D>> vectorShapeActors;
    std::vector<vtkSmartPointer<vtkPolyDataMapper2D>> vectorShapeMappers;
    std::vector<vtkSmartPointer<vtkPolyData>> vectorShapePolyDatas;
    std::vector<vtkSmartPointer<vtkPoints>> vectorShapePointsList;
    
    // 控制点显示相关
    vtkSmartPointer<vtkPoints> controlPoints;
    vtkSmartPointer<vtkPolyData> controlPolyData;
    vtkSmartPointer<vtkActor2D> controlActor;
    vtkSmartPointer<vtkPolyDataMapper2D> controlMapper;
    
    // 当前正在绘制的形状显示（绘制模式）
    vtkSmartPointer<vtkPoints> currentShapePoints;
    vtkSmartPointer<vtkPolyData> currentShapePolyData;
    vtkSmartPointer<vtkActor2D> currentShapeActor;
    vtkSmartPointer<vtkPolyDataMapper2D> currentShapeMapper;
    
    bool drawingModeEnabled;
    bool isDrawing;
    int startX, startY;
    int currentX, currentY;
    
    // 当前绘制的形状类型
    SelectionShape currentDrawingShape;
    
    // 多边形绘制状态
    bool isDrawingPolygon;
    std::vector<std::pair<int, int>> currentPolygonVertices;
    
    // 矢量图形存储
    std::vector<std::unique_ptr<VectorShape>> vectorShapes;
    
    // 编辑状态
    VectorShape* selectedShape;
    EditOperation currentOperation;
    int selectedControlPoint;
    bool isDragging;
    double lastMouseX, lastMouseY;
    
    // 撤销系统
    std::stack<std::unique_ptr<Command>> commandHistory;
    static const size_t MAX_HISTORY_SIZE = 50;
    
    // 拖拽前的状态（用于撤销）
    std::string shapeStateBeforeDrag;

    std::function<void(Qt::CursorShape)> cursorCallback;
    
    // 选中的点ID列表
    std::vector<vtkIdType> selectedPointIds;
    
    // 原始颜色备份
    std::vector<unsigned char> originalColorBackup;

    // 遮挡检测标志
    bool occlusionDetectionEnabled;
    
    // 视图锁定标志
    bool viewLocked;

    // 图形融合相关的新方法
    void PerformShapeUnion();
    std::vector<std::pair<double, double>> ConvertShapeToPolygon(const Shape& shape, int subdivisions = 64);
    std::vector<std::pair<double, double>> UnionPolygons(const std::vector<std::vector<std::pair<double, double>>>& polygons);
    std::vector<std::pair<double, double>> ComputeConvexHull(const std::vector<std::pair<double, double>>& points);
    std::vector<std::pair<double, double>> MergePolygonsByGrid(const std::vector<std::vector<std::pair<double, double>>>& polygons);
    bool IsPointInsideAnyPolygon(double x, double y, const std::vector<std::vector<std::pair<double, double>>>& polygons);
    std::vector<std::pair<double, double>> TraceBoundary(const std::vector<std::vector<bool>>& grid, int gridWidth, int gridHeight, double minX, double minY, double stepX, double stepY);
    double CrossProduct(const std::pair<double, double>& O, const std::pair<double, double>& A, const std::pair<double, double>& B);
    std::vector<std::pair<double, double>> SortBoundaryPoints(const std::vector<std::pair<double, double>>& points);
};

#endif // SELECTOR_H 