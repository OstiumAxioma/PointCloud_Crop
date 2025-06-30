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
class PointCloudSelector;
class SelectionState;

//=============================================================================
// 命令模式 - 统一的状态变更命令
//=============================================================================

// 命令基类
class Command {
public:
    virtual ~Command() = default;
    virtual void execute() = 0;
    virtual void undo() = 0;
    virtual std::string getDescription() const = 0;
};

// 统一的状态变更命令（替代原有的MoveShapeCommand和EditShapeCommand）
class StateChangeCommand : public Command {
public:
    StateChangeCommand(VectorShape* shape, const std::string& beforeState, const std::string& afterState, const std::string& description = "状态变更");
    void execute() override;
    void undo() override;
    std::string getDescription() const override;

private:
    VectorShape* shape_;
    std::string beforeState_;
    std::string afterState_;
    std::string description_;
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

//=============================================================================
// 形状类层次结构 - 移除Shape类冗余
//=============================================================================

// 矢量图形基类 - 现在包含所有必要功能，不再需要Shape类
class VectorShape {
public:
    virtual ~VectorShape() = default;
    
    // 绘制图形
    virtual void draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, vtkPolyData* polyData, vtkPoints* points) = 0;
    
    // 碰撞检测
    virtual bool hitTest(double x, double y, double tolerance = 8.0) const = 0;
    
    // 直接的点包含检测（替代原有的Shape::containsPoint）
    virtual bool containsPoint(double x, double y) const = 0;
    
    // 控制点碰撞检测
    virtual bool hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance = 8.0) const = 0;
    
    // 拖拽操作
    virtual void startDrag(double x, double y) = 0;
    virtual void updateDrag(double x, double y, bool shiftPressed = false) = 0;
    virtual void endDrag() = 0;
    
    // 移动控制点
    virtual void moveControlPoint(int index, double x, double y) = 0;
    
    // 获取形状类型
    virtual SelectionShape getType() const = 0;
    
    // 序列化状态（用于撤销功能）
    virtual std::string serialize() const = 0;
    virtual void deserialize(const std::string& data) = 0;
    
    // 选择状态
    virtual void setSelected(bool selected) { isSelected_ = selected; }
    virtual bool isSelected() const { return isSelected_; }
    
    // 为了向后兼容保留的Shape结构体
    struct ShapeData {
        SelectionShape type;
        union {
            struct { double x1, y1, x2, y2; } rect;
            struct { double centerX, centerY, radius; } circle;
            struct { std::vector<std::pair<double, double>>* vertices; } polygon;
        };
        std::vector<std::pair<double, double>> polygonVertices; // 用于多边形顶点存储
        
        ShapeData(SelectionShape t) : type(t) {
            if (type == SelectionShape::Polygon) {
                polygon.vertices = &polygonVertices;
            }
        }
    };
    
    // 转换为兼容格式（保持向后兼容）
    virtual ShapeData toShapeData() const = 0;
    
protected:
    bool isSelected_ = false;
};

// 矢量矩形
class VectorRectangle : public VectorShape {
public:
    VectorRectangle(double x1, double y1, double x2, double y2);
    
    void draw(vtkRenderer* renderer, vtkActor2D* actor, vtkPolyDataMapper2D* mapper, vtkPolyData* polyData, vtkPoints* points) override;
    bool hitTest(double x, double y, double tolerance = 8.0) const override;
    bool containsPoint(double x, double y) const override;
    bool hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance = 8.0) const override;
    void startDrag(double x, double y) override;
    void updateDrag(double x, double y, bool shiftPressed = false) override;
    void endDrag() override;
    void moveControlPoint(int index, double x, double y) override;
    SelectionShape getType() const override { return SelectionShape::Rectangle; }
    ShapeData toShapeData() const override;
    
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
    bool containsPoint(double x, double y) const override;
    bool hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance = 8.0) const override;
    void startDrag(double x, double y) override;
    void updateDrag(double x, double y, bool shiftPressed = false) override;
    void endDrag() override;
    void moveControlPoint(int index, double x, double y) override;
    SelectionShape getType() const override { return SelectionShape::Circle; }
    ShapeData toShapeData() const override;
    
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
    bool containsPoint(double x, double y) const override;
    bool hitTestControlPoint(double x, double y, int& controlPointIndex, double tolerance = 8.0) const override;
    void startDrag(double x, double y) override;
    void updateDrag(double x, double y, bool shiftPressed = false) override;
    void endDrag() override;
    void moveControlPoint(int index, double x, double y) override;
    SelectionShape getType() const override { return SelectionShape::Polygon; }
    ShapeData toShapeData() const override;
    
    std::string serialize() const override;
    void deserialize(const std::string& data) override;
    
private:
    std::vector<std::pair<double, double>> vertices_; // 顶点列表
    double dragStartX_, dragStartY_; // 拖拽起始点
    std::vector<std::pair<double, double>> originalVertices_; // 拖拽前的原始顶点
    
    bool isPointInPolygon(double x, double y) const; // 判断点是否在多边形内
};

//=============================================================================
// 点云选择模块 - 模块化选择逻辑
//=============================================================================

class PointCloudSelector {
public:
    PointCloudSelector(vtkRenderer* renderer, vtkPolyData* pointData);
    ~PointCloudSelector() = default;
    
    // 设置数据
    void setRenderer(vtkRenderer* renderer) { renderer_ = renderer; }
    void setPointCloudData(vtkPolyData* pointData) { originalPointData_ = pointData; }
    void setOcclusionDetectionEnabled(bool enabled) { occlusionDetectionEnabled_ = enabled; }
    
    // 主要选择方法
    std::vector<vtkIdType> selectPointsByShapes(const std::vector<VectorShape*>& shapes);
    void highlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds);
    void clearAllSelectedPoints();
    
    // 获取当前选中的点
    const std::vector<vtkIdType>& getSelectedPoints() const { return selectedPointIds_; }
    void addSelectedPoints(const std::vector<vtkIdType>& points);
    
private:
    // 核心选择算法
    std::vector<vtkIdType> collectCandidatePoints(const std::vector<VectorShape*>& shapes);
    std::vector<vtkIdType> filterOccludedPoints(const std::vector<vtkIdType>& candidatePoints,
                                               const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                                               const std::map<vtkIdType, double>& distancesToCamera);
    
    // 辅助方法
    bool isPointInShapes(double screenX, double screenY, const std::vector<VectorShape*>& shapes);
    double calculateScreenDistance(double x1, double y1, double x2, double y2);
    bool isPointOccluded(vtkIdType pointId, const std::vector<vtkIdType>& frontPoints, 
                        const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                        double occlusionThreshold = 5.0);
    
    // 成员变量
    vtkRenderer* renderer_;
    vtkPolyData* originalPointData_;
    bool occlusionDetectionEnabled_;
    std::vector<vtkIdType> selectedPointIds_;
    std::vector<unsigned char> originalColorBackup_;
};

//=============================================================================
// 状态模式 - 改进状态管理
//=============================================================================

// 选择状态基类
class SelectionState {
public:
    virtual ~SelectionState() = default;
    virtual void handleMouseDown(Selector* selector, int x, int y) = 0;
    virtual void handleMouseMove(Selector* selector, int x, int y) = 0;
    virtual void handleMouseUp(Selector* selector) = 0;
    virtual void handleRightClick(Selector* selector) = 0;
    virtual void handleKeyPress(Selector* selector, const std::string& key, bool ctrlPressed) = 0;
    virtual std::string getStateName() const = 0;
};

// 绘制状态
class DrawingState : public SelectionState {
public:
    void handleMouseDown(Selector* selector, int x, int y) override;
    void handleMouseMove(Selector* selector, int x, int y) override;
    void handleMouseUp(Selector* selector) override;
    void handleRightClick(Selector* selector) override;
    void handleKeyPress(Selector* selector, const std::string& key, bool ctrlPressed) override;
    std::string getStateName() const override { return "Drawing"; }
};

// 编辑状态
class EditingState : public SelectionState {
public:
    void handleMouseDown(Selector* selector, int x, int y) override;
    void handleMouseMove(Selector* selector, int x, int y) override;
    void handleMouseUp(Selector* selector) override;
    void handleRightClick(Selector* selector) override;
    void handleKeyPress(Selector* selector, const std::string& key, bool ctrlPressed) override;
    std::string getStateName() const override { return "Editing"; }
};

// 多边形绘制状态
class PolygonDrawingState : public SelectionState {
public:
    void handleMouseDown(Selector* selector, int x, int y) override;
    void handleMouseMove(Selector* selector, int x, int y) override;
    void handleMouseUp(Selector* selector) override;
    void handleRightClick(Selector* selector) override;
    void handleKeyPress(Selector* selector, const std::string& key, bool ctrlPressed) override;
    std::string getStateName() const override { return "PolygonDrawing"; }
};

//=============================================================================
// 主要的Selector类 - 重构后的版本
//=============================================================================

class Selector : public vtkInteractorStyleTrackballCamera
{
public:
    static Selector* New();
    vtkTypeMacro(Selector, vtkInteractorStyleTrackballCamera);

    // 设置渲染器和点云数据
    void SetRenderer(vtkRenderer* renderer);
    void SetPointCloudData(vtkPolyData* pointData);
    
    // 模式控制
    void EnableDrawingMode(bool enable);
    bool IsDrawingModeEnabled() const;
    
    void EnableOcclusionDetection(bool enable);
    bool IsOcclusionDetectionEnabled() const;
    
    void EnableViewLock(bool enable) { viewLocked_ = enable; }
    bool IsViewLocked() const { return viewLocked_; }

    // 绘制形状控制
    void SetCurrentDrawingShape(SelectionShape shape);
    SelectionShape GetCurrentDrawingShape() const { return currentDrawingShape_; }

    // 画布操作
    void ClearCanvas();
    void ConfirmSelection();
    size_t GetCanvasShapeCount() const { return vectorShapes_.size(); }
    
    // 选择操作
    void ClearAllSelectedPoints();
    void ClearCurrentDrawing();
    size_t GetSelectedPointCount() const;
    
    // 光标控制
    void SetCursorCallback(std::function<void(Qt::CursorShape)> callback) {
        cursorCallback_ = callback;
    }
    
    // 撤销系统
    void ExecuteCommand(std::unique_ptr<Command> command);
    void Undo();
    bool CanUndo() const { return !commandHistory_.empty(); }
    
    // 图形编辑
    void DeleteSelectedShape();
    
    // 内部方法（供命令类调用）
    void AddShapeInternal(std::unique_ptr<VectorShape> shape);
    std::unique_ptr<VectorShape> RemoveShapeInternal(VectorShape* shape);
    void InsertShapeInternal(std::unique_ptr<VectorShape> shape, size_t index);
    size_t GetShapeIndex(VectorShape* shape) const;
    
    // 状态管理
    void SetState(std::unique_ptr<SelectionState> state);
    SelectionState* GetCurrentState() const { return currentState_.get(); }
    
    // 绘制方法（状态类需要访问）
    void DrawCurrentShape();
    void DrawRectangle(double x1, double y1, double x2, double y2);
    void DrawCircle(double centerX, double centerY, double radius);
    void DrawPolygon(const std::vector<std::pair<double, double>>& vertices, bool addTemporaryVertex = true);
    void AddPolygonVertex(int x, int y);
    void UndoLastVertex();
    void CompleteCurrentPolygon();
    
    // 图形管理
    void UpdateVectorShapeDisplay();
    void DrawControlPoints();
    VectorShape* GetShapeAtPosition(double x, double y);
    void SelectShape(VectorShape* shape);
    void DeselectAllShapes();
    
    // 访问内部状态（供状态类使用）
    bool& IsDrawing() { return isDrawing_; }
    int& StartX() { return startX_; }
    int& StartY() { return startY_; }
    int& CurrentX() { return currentX_; }
    int& CurrentY() { return currentY_; }
    std::vector<std::pair<int, int>>& CurrentPolygonVertices() { return currentPolygonVertices_; }
    VectorShape*& SelectedShape() { return selectedShape_; }
    EditOperation& CurrentOperation() { return currentOperation_; }
    int& SelectedControlPoint() { return selectedControlPoint_; }
    bool& IsDragging() { return isDragging_; }
    double& LastMouseX() { return lastMouseX_; }
    double& LastMouseY() { return lastMouseY_; }
    std::string& ShapeStateBeforeDrag() { return shapeStateBeforeDrag_; }
    vtkSmartPointer<vtkActor2D>& CurrentShapeActor() { return currentShapeActor_; }

protected:
    Selector();
    ~Selector();

    // VTK事件处理（委托给状态）
    virtual void OnLeftButtonDown() override;
    virtual void OnLeftButtonUp() override;
    virtual void OnRightButtonDown() override;
    virtual void OnMouseMove() override;
    virtual void OnKeyPress() override;

private:
    // 核心组件
    vtkSmartPointer<vtkRenderer> renderer_;
    std::unique_ptr<PointCloudSelector> pointCloudSelector_;
    std::unique_ptr<SelectionState> currentState_;
    
    // 图形存储
    std::vector<std::unique_ptr<VectorShape>> vectorShapes_;
    
    // VTK显示组件
    std::vector<vtkSmartPointer<vtkActor2D>> vectorShapeActors_;
    std::vector<vtkSmartPointer<vtkPolyDataMapper2D>> vectorShapeMappers_;
    std::vector<vtkSmartPointer<vtkPolyData>> vectorShapePolyDatas_;
    std::vector<vtkSmartPointer<vtkPoints>> vectorShapePointsList_;
    
    // 控制点显示
    vtkSmartPointer<vtkPoints> controlPoints_;
    vtkSmartPointer<vtkPolyData> controlPolyData_;
    vtkSmartPointer<vtkActor2D> controlActor_;
    vtkSmartPointer<vtkPolyDataMapper2D> controlMapper_;
    
    // 当前绘制形状显示
    vtkSmartPointer<vtkPoints> currentShapePoints_;
    vtkSmartPointer<vtkPolyData> currentShapePolyData_;
    vtkSmartPointer<vtkActor2D> currentShapeActor_;
    vtkSmartPointer<vtkPolyDataMapper2D> currentShapeMapper_;
    
    // 状态变量（现在由状态类管理）
    SelectionShape currentDrawingShape_;
    bool isDrawing_;
    int startX_, startY_;
    int currentX_, currentY_;
    std::vector<std::pair<int, int>> currentPolygonVertices_;
    
    // 编辑状态
    VectorShape* selectedShape_;
    EditOperation currentOperation_;
    int selectedControlPoint_;
    bool isDragging_;
    double lastMouseX_, lastMouseY_;
    std::string shapeStateBeforeDrag_;
    
    // 系统状态
    bool viewLocked_;
    std::function<void(Qt::CursorShape)> cursorCallback_;
    
    // 撤销系统
    std::stack<std::unique_ptr<Command>> commandHistory_;
    static const size_t MAX_HISTORY_SIZE = 50;
    
    // 默认状态实例
    std::unique_ptr<DrawingState> drawingState_;
    std::unique_ptr<EditingState> editingState_;
    std::unique_ptr<PolygonDrawingState> polygonDrawingState_;
    
    // 清理方法
    void ClearCurrentShapeDisplay();
    
    // 向后兼容方法（将逐步迁移到PointCloudSelector）
    void PerformCanvasBasedSelection();
    void PerformCanvasBasedSelection(const std::vector<VectorShape*>& shapes);
};

#endif // SELECTOR_H 