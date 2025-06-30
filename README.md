# 点云处理应用

这是一个基于VTK 8.2.0和Qt 5.12.9的点云可视化与处理应用，支持PLY文件导入、3D可视化、多种形状选择（矩形、圆形、多边形）和点云高亮功能。

## 项目结构

```
PointCloud_Crop/
├── CMakeLists.txt          # CMake配置文件
├── build.bat              # Windows构建脚本
├── generate_vscode_config.py # VSCode配置生成脚本
├── README.md              # 项目说明
├── header/                # 头文件目录
│   ├── mainwindow.h       # 主窗口头文件
│   ├── pointcloudviewer.h # 点云查看器头文件
│   ├── fileimporter.h     # 文件导入器头文件
│   └── selector.h         # 多形状选择器头文件
└── src/                   # 源文件目录
    ├── main.cpp           # 主程序入口
    ├── mainwindow.cpp     # 主窗口实现
    ├── mainwindow.ui      # Qt Designer UI文件
    ├── pointcloudviewer.cpp # 点云查看器实现
    ├── fileimporter.cpp   # 文件导入器实现
    └── selector.cpp       # 多形状选择器实现
```

## 环境要求

- **VTK**: 8.2.0 
  - 源码位置: `D:\code\vtk8.2.0\VTK-8.2.0\`
  - 编译输出: `D:\code\vtk8.2.0\VTK-8.2.0\bin`
  - CMake配置: `D:\code\vtk8.2.0\VTK-8.2.0\lib\cmake\vtk-8.2`
- **Qt**: 5.12.9 (安装在 `C:\Qt\Qt5.12.9\5.12.9\msvc2017_64`)
- **编译器**: Visual Studio 2022 或更高版本
- **CMake**: 3.14 或更高版本

## 最新更新

### v2.0 功能增强
- ✅ **多形状选择**: 新增圆形和多边形选择模式
- ✅ **智能遮挡检测**: 基于深度缓冲的自动遮挡检测
- ✅ **交互式多边形绘制**: 支持逐点绘制，Backspace撤销
- ✅ **累积选择**: 多次选择结果累积，不覆盖
- ✅ **状态管理**: 自动清理选择状态，优化用户体验
- ✅ **射线法算法**: 高精度多边形内点判断
- ✅ **实时预览**: 绘制过程中实时显示选择框
- ✅ **键盘交互**: 支持Backspace撤销操作

### v3.0 矢量图形编辑系统
- ✅ **CAD风格编辑**: 图形绘制后可进行类似CAD的编辑操作
- ✅ **图形选择**: 点击图形变红色高亮，显示编辑控制点
- ✅ **矩形编辑**: 拖拽控制点调整大小，保持90度角不变
- ✅ **圆形编辑**: 拖拽控制点调整半径，支持Shift键保持比例
- ✅ **多边形编辑**: 点击显示顶点控制点，拖拽顶点改变形状
- ✅ **图形移动**: 拖拽整个图形进行位置移动
- ✅ **撤销系统**: Ctrl+Z撤销所有操作（添加、编辑、删除）
- ✅ **删除功能**: Del键删除选中图形（可撤销）
- ✅ **独立渲染**: 每个图形独立actor，选择状态互不影响
- ✅ **状态序列化**: 完整的图形状态保存和恢复机制

## 构建步骤

### 方法1: 使用构建脚本 (推荐)
```bash
# 在项目根目录运行
build.bat
```

### 方法2: 手动构建
```bash
# 创建构建目录
mkdir build
cd build

# 配置项目
cmake .. -G "Visual Studio 17 2022" -A x64

# 构建项目
cmake --build . --config Release
```

## 功能特性

- **PLY文件导入**: 支持导入PLY格式的点云文件
- **3D可视化**: 使用VTK进行点云3D渲染，支持Z轴渐变色显示
- **多形状选择**: 支持矩形、圆形、多边形三种选择模式
- **智能选择**: 支持遮挡检测，优先选择距离相机较近的点
- **交互式绘制**: 多边形模式支持逐点绘制，Backspace撤销
- **矢量图形编辑**: CAD风格的图形创建和编辑系统
- **图形选择与编辑**: 点击图形进行选择，拖拽控制点编辑形状
- **撤销与删除**: 完整的撤销系统，支持Ctrl+Z和Del键操作
- **点云高亮**: 选中的点以红色高亮显示
- **累积选择**: 支持多次选择，选中点累积高亮
- **交互式操作**: 支持鼠标旋转、平移、缩放3D场景
- **Qt界面**: 现代化的Qt用户界面，包含菜单、工具栏和状态栏

## 核心功能

### 点云导入与显示
- 通过菜单或工具栏导入PLY文件
- 自动根据Z坐标生成渐变色显示
- 支持点云统计信息显示（点数、面片数、Z范围）

### 多形状选择功能

#### 矩形选择
- 点击"矩形选择"按钮启用矩形选择模式
- 鼠标拖拽绘制矩形选择框
- 实时显示选择框，不随3D场景旋转

#### 圆形选择
- 点击"圆形选择"按钮启用圆形选择模式
- 鼠标拖拽绘制圆形选择区域
- 以拖拽起点和终点为对角线的矩形的内切圆为选择区域

#### 多边形选择
- 点击"多边形选择"按钮启用多边形选择模式
- 左键依次点击放置多边形顶点
- Backspace键撤销最后一个顶点
- 右键完成多边形绘制并执行选择
- 支持任意复杂形状的精确选择

#### 遮挡检测
- 默认启用深度选择功能
- 自动检测点云遮挡关系，优先选择距离相机较近的点
- 可通过工具栏复选框开启/关闭此功能

#### 选择特性
- 支持多次选择，选中点累积高亮
- 所有选择模式互斥，激活一种会自动关闭其他模式
- 切换选择模式时自动清理当前绘制状态

### 矢量图形编辑功能

#### 图形状态管理
- **正常状态**: 绿色线条显示，表示可用于选取的图形
- **选中状态**: 红色线条显示，表示当前编辑的图形
- **控制点显示**: 选中图形时显示黄色控制点，用于调整形状

#### 图形选择
- **点击选择**: 点击任意图形边界进行选择
- **状态切换**: 选择新图形时，之前选中的图形自动变回绿色
- **取消选择**: 点击空白区域取消所有选择
- **独立渲染**: 每个图形有独立的渲染对象，选择状态互不影响

#### 矩形编辑
- **角点控制**: 四个角点控制矩形的位置和大小
- **边中点控制**: 四个边中点控制矩形的宽度或高度
- **约束编辑**: 保持90度角不变，只允许拉伸操作
- **实时预览**: 拖拽过程中实时显示调整结果

#### 圆形编辑
- **半径控制**: 四个方向的控制点（上下左右）
- **中心固定**: 拖拽控制点调整半径，圆心位置保持不变
- **比例缩放**: 所有控制点功能相同，调整圆的大小
- **移动操作**: 拖拽圆的边界进行整体移动

#### 多边形编辑
- **顶点控制**: 每个顶点都是控制点，可以独立移动
- **形状变化**: 拖拽顶点改变多边形的形状
- **无缩放功能**: 不支持整体缩放，只支持顶点移动
- **自由编辑**: 可以创建任意复杂的多边形形状

#### 图形移动
- **整体拖拽**: 点击图形边界（非控制点）拖拽整个图形
- **位置保持**: 移动时保持图形的大小和形状不变
- **实时反馈**: 移动过程中实时更新图形位置

#### 键盘快捷键
- **Ctrl+Z**: 撤销上一个操作（添加、编辑、删除）
- **Del键**: 删除当前选中的图形（红色图形）
- **Backspace**: 多边形绘制时撤销最后一个顶点

#### 撤销系统
- **操作记录**: 自动记录所有图形操作（最多50个）
- **状态恢复**: 撤销时完整恢复图形的位置、大小、形状
- **命令模式**: 基于命令模式实现，支持复杂操作的撤销
- **类型支持**: 
  - 图形添加/删除
  - 图形移动
  - 图形形状编辑（顶点移动、缩放等）

### 选择管理
- 选中点以红色高亮显示
- 支持清除所有选中点，恢复原始颜色
- 选中状态持久保存，直到手动清除

## 核心技术功能

### 1. 深度选取算法

#### 算法架构
深度选取系统由`PointCloudSelector`类实现，提供基于深度缓冲的智能点云选择功能。

**核心类**: `PointCloudSelector`
```cpp
class PointCloudSelector {
private:
    vtkRenderer* renderer_;
    vtkPolyData* originalPointData_;
    bool occlusionDetectionEnabled_;
    std::vector<vtkIdType> selectedPointIds_;
    std::vector<unsigned char> originalColorBackup_;

public:
    std::vector<vtkIdType> selectPointsByShapes(const std::vector<VectorShape*>& shapes);
    void highlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds);
    void clearAllSelectedPoints();
};
```

#### 选择流程
1. **候选点收集**: `collectCandidatePoints(shapes)` - 屏幕空间投影判断
2. **深度过滤**: `filterOccludedPoints()` - 基于深度缓冲的遮挡检测
3. **结果高亮**: `highlightSelectedPoints()` - 直接修改VTK颜色数据

**关键算法实现**:
```cpp
std::vector<vtkIdType> PointCloudSelector::filterOccludedPoints(
    const std::vector<vtkIdType>& candidatePoints,
    const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
    const std::map<vtkIdType, double>& distancesToCamera) {
    
    // 1. 按距离排序（近到远）
    std::sort(sortedPoints.begin(), sortedPoints.end(), 
              [&](vtkIdType a, vtkIdType b) {
                  return distancesToCamera.at(a) < distancesToCamera.at(b);
              });
    
    // 2. 深度缓冲检测
    std::map<std::pair<int, int>, double> depthBuffer;
    for (vtkIdType pointId : sortedPoints) {
        if (!isOccluded(pointId, depthBuffer)) {
            visiblePoints.push_back(pointId);
            updateDepthBuffer(pointId, depthBuffer);
        }
    }
    return visiblePoints;
}
```

#### 几何算法
- **矩形**: AABB包围盒检测 `(x >= x1 && x <= x2 && y >= y1 && y <= y2)`
- **圆形**: 欧几里得距离判断 `sqrt((x-cx)² + (y-cy)²) <= radius`
- **多边形**: 射线法算法 `isPointInPolygon(x, y)`

#### 数据流
```
3D点云 → 屏幕投影 → 形状内判断 → 深度检测 → 选中点ID → 颜色高亮
```

### 2. 选择框画布系统

#### 类关系架构
```
Selector (协调器)
├── PointCloudSelector (点云选择)
├── SelectionState (状态管理)
│   ├── DrawingState
│   ├── EditingState  
│   └── PolygonDrawingState
└── VectorShape (图形对象)
    ├── VectorRectangle
    ├── VectorCircle
    └── VectorPolygon
```

#### 核心接口定义
**VectorShape基类**:
```cpp
class VectorShape {
public:
    virtual bool containsPoint(double x, double y) const = 0;
    virtual void draw(vtkRenderer* renderer, vtkActor2D* actor, ...) = 0;
    virtual bool hitTest(double x, double y, double tolerance = 8.0) const = 0;
    virtual bool hitTestControlPoint(double x, double y, int& index, double tolerance = 8.0) const = 0;
    
    // 编辑操作
    virtual void startDrag(double x, double y) = 0;
    virtual void updateDrag(double x, double y, bool shiftPressed = false) = 0;
    virtual void endDrag() = 0;
    virtual void moveControlPoint(int index, double x, double y) = 0;
    
    // 状态序列化
    virtual std::string serialize() const = 0;
    virtual void deserialize(const std::string& data) = 0;
};
```

**状态管理接口**:
```cpp
class SelectionState {
public:
    virtual void handleMouseDown(Selector* selector, int x, int y) = 0;
    virtual void handleMouseMove(Selector* selector, int x, int y) = 0;
    virtual void handleMouseUp(Selector* selector) = 0;
    virtual void handleRightClick(Selector* selector) = 0;
    virtual void handleKeyPress(Selector* selector, const std::string& key, bool ctrl) = 0;
};
```

#### 渲染系统
**独立渲染对象**:
```cpp
// 每个图形的独立VTK组件
struct ShapeRenderData {
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkPolyData> polyData;
    vtkSmartPointer<vtkPolyDataMapper2D> mapper;
    vtkSmartPointer<vtkActor2D> actor;
};

// 管理容器
std::vector<std::unique_ptr<VectorShape>> vectorShapes_;
std::vector<vtkSmartPointer<vtkActor2D>> vectorShapeActors_;
```

#### 交互处理流程
```cpp
// 事件委托模式
void Selector::OnLeftButtonDown() {
    if (!IsDrawingModeEnabled()) {
        // 编辑模式：直接处理
        handleEditingMode(x, y);
    } else {
        // 绘制模式：委托给状态类
        currentState_->handleMouseDown(this, x, y);
    }
}
```

#### 图形编辑接口
**控制点系统**:
```cpp
// 矩形：8个控制点（4角+4边中点）
std::vector<std::pair<double, double>> VectorRectangle::getControlPoints() const {
    return {
        {x1_, y1_}, {x2_, y1_}, {x2_, y2_}, {x1_, y2_},           // 四个角
        {(x1_+x2_)/2, y1_}, {x2_, (y1_+y2_)/2},                  // 边中点
        {(x1_+x2_)/2, y2_}, {x1_, (y1_+y2_)/2}
    };
}

// 圆形：4个方向控制点
// 多边形：每个顶点一个控制点
```

### 3. 撤销系统与序列化

#### 命令模式架构
```cpp
class Command {
public:
    virtual void execute() = 0;
    virtual void undo() = 0;
    virtual std::string getDescription() const = 0;
};

// 统一状态变更命令
class StateChangeCommand : public Command {
private:
    VectorShape* shape_;
    std::string beforeState_;  // 序列化的前状态
    std::string afterState_;   // 序列化的后状态
    
public:
    void execute() override { shape_->deserialize(afterState_); }
    void undo() override { shape_->deserialize(beforeState_); }
};
```

#### 序列化实现
**图形状态序列化**:
```cpp
// VectorRectangle序列化
std::string VectorRectangle::serialize() const {
    return std::to_string(x1_) + "," + std::to_string(y1_) + "," + 
           std::to_string(x2_) + "," + std::to_string(y2_);
}

void VectorRectangle::deserialize(const std::string& data) {
    auto tokens = split(data, ',');
    x1_ = std::stod(tokens[0]);
    y1_ = std::stod(tokens[1]);
    x2_ = std::stod(tokens[2]);
    y2_ = std::stod(tokens[3]);
    normalizeCoordinates();
}

// VectorCircle序列化
std::string VectorCircle::serialize() const {
    return std::to_string(centerX_) + "," + std::to_string(centerY_) + "," + 
           std::to_string(radius_);
}

// VectorPolygon序列化（顶点坐标列表）
std::string VectorPolygon::serialize() const {
    std::string result;
    for (size_t i = 0; i < vertices_.size(); ++i) {
        if (i > 0) result += ";";
        result += std::to_string(vertices_[i].first) + "," + 
                 std::to_string(vertices_[i].second);
    }
    return result;
}
```

#### 撤销系统管理
```cpp
class Selector {
private:
    std::stack<std::unique_ptr<Command>> commandHistory_;
    static const size_t MAX_HISTORY_SIZE = 50;
    
public:
    void ExecuteCommand(std::unique_ptr<Command> command) {
        command->execute();
        commandHistory_.push(std::move(command));
        
        // 限制历史大小
        while (commandHistory_.size() > MAX_HISTORY_SIZE) {
            // 移除最老的命令
        }
    }
    
    void Undo() {
        if (!commandHistory_.empty()) {
            auto command = std::move(commandHistory_.top());
            commandHistory_.pop();
            command->undo();
        }
    }
};
```

#### 编辑操作捕获
```cpp
// 编辑前记录状态
void EditingState::handleMouseDown(Selector* selector, int x, int y) {
    if (selector->SelectedShape()) {
        // 记录编辑前状态
        std::string beforeState = selector->SelectedShape()->serialize();
        selector->ShapeStateBeforeDrag() = beforeState;
        selector->SelectedShape()->startDrag(x, y);
    }
}

// 编辑完成创建命令
void EditingState::handleMouseUp(Selector* selector) {
    if (selector->SelectedShape()) {
        std::string afterState = selector->SelectedShape()->serialize();
        if (afterState != selector->ShapeStateBeforeDrag()) {
            auto command = std::make_unique<StateChangeCommand>(
                selector->SelectedShape(),
                selector->ShapeStateBeforeDrag(),
                afterState,
                "编辑图形"
            );
            selector->ExecuteCommand(std::move(command));
        }
    }
}
```

#### 数据流总览
```
用户操作 → 状态记录 → 图形修改 → 序列化状态 → 创建命令 → 历史栈
    ↓
撤销触发 → 命令出栈 → 反序列化 → 恢复状态 → 刷新显示
```

## 集成接口

### 主要集成点
1. **PointCloudSelector**: 独立的点云选择模块，可直接集成
2. **VectorShape系列**: 矢量图形对象，支持序列化传输
3. **Command系统**: 完整的撤销功能，可扩展到更大的命令系统
4. **SelectionState**: 状态管理模式，可扩展更多交互模式

### 数据格式
- **点云数据**: 标准VTK PolyData格式
- **图形序列化**: 文本格式，易于存储和传输
- **选中点**: vtkIdType数组，兼容VTK生态

### 扩展能力
- **新图形类型**: 继承VectorShape即可
- **新选择算法**: 扩展PointCloudSelector方法
- **新交互模式**: 继承SelectionState实现
- **自定义命令**: 继承Command接口

## 代码重构与架构优化

### 重构目标与成果

本项目经历了全面的架构重构，从原有的"过程式+面向对象"混合架构转换为更纯粹的"面向对象+设计模式"架构。重构实现了以下四个核心目标：

#### 1. Shape类冗余消除 ✅
**问题**: 原架构中既有Shape类（简单数据容器）又有VectorShape类（功能完整的对象），存在双重表示

**解决方案**:
- 移除复杂的Shape类（约150行union结构代码）
- 直接在VectorShape子类中实现`containsPoint()`方法
- 保留`ShapeData`结构用于向后兼容

**关键实现**:
```cpp
// VectorRectangle中的点包含检测
bool VectorRectangle::containsPoint(double x, double y) const {
    double x1 = std::min(x1_, x2_);
    double x2 = std::max(x1_, x2_);
    double y1 = std::min(y1_, y2_);
    double y2 = std::max(y1_, y2_);
    return (x >= x1 && x <= x2 && y >= y1 && y <= y2);
}

// VectorCircle中的点包含检测
bool VectorCircle::containsPoint(double x, double y) const {
    double distance = sqrt(pow(x - centerX_, 2) + pow(y - centerY_, 2));
    return distance <= radius_;
}

// VectorPolygon中的点包含检测（射线法）
bool VectorPolygon::containsPoint(double x, double y) const {
    return isPointInPolygon(x, y);
}
```

#### 2. 命令类合并 ✅
**问题**: MoveShapeCommand和EditShapeCommand本质上都是修改形状状态，存在重复

**解决方案**:
- 统一为StateChangeCommand类
- 通过序列化/反序列化实现状态保存和恢复

**关键实现**:
```cpp
class StateChangeCommand : public Command {
public:
    StateChangeCommand(VectorShape* shape, 
                      const std::string& beforeState, 
                      const std::string& afterState, 
                      const std::string& description = "状态变更");
    
    void execute() override { shape_->deserialize(afterState_); }
    void undo() override { shape_->deserialize(beforeState_); }
    std::string getDescription() const override { return description_; }

private:
    VectorShape* shape_;
    std::string beforeState_;
    std::string afterState_;
    std::string description_;
};
```

#### 3. 选择逻辑模块化 ✅
**问题**: 点云选择和遮挡检测代码散布在Selector类中，职责不清

**解决方案**:
- 创建专门的PointCloudSelector类（约300行代码）
- 提取所有点云选择算法到独立模块

**关键实现**:
```cpp
class PointCloudSelector {
public:
    PointCloudSelector(vtkRenderer* renderer, vtkPolyData* pointData);
    
    // 主要选择方法
    std::vector<vtkIdType> selectPointsByShapes(const std::vector<VectorShape*>& shapes);
    void highlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds);
    void clearAllSelectedPoints();
    
    // 高级功能
    void setOcclusionDetectionEnabled(bool enabled);
    const std::vector<vtkIdType>& getSelectedPoints() const;

private:
    // 核心算法
    std::vector<vtkIdType> collectCandidatePoints(const std::vector<VectorShape*>& shapes);
    std::vector<vtkIdType> filterOccludedPoints(const std::vector<vtkIdType>& candidatePoints,
                                               const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
                                               const std::map<vtkIdType, double>& distancesToCamera);
    bool isPointInShapes(double screenX, double screenY, const std::vector<VectorShape*>& shapes);
};
```

#### 4. 状态管理改进 ✅
**问题**: 状态管理分散在多个布尔变量和枚举中，难以维护

**解决方案**:
- 实现状态模式，用状态类管理不同交互模式
- 清晰的状态转换和事件处理

**关键实现**:
```cpp
// 状态基类
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

// 绘制状态实现
class DrawingState : public SelectionState {
public:
    void handleMouseDown(Selector* selector, int x, int y) override;
    void handleMouseMove(Selector* selector, int x, int y) override;
    void handleMouseUp(Selector* selector) override;
    std::string getStateName() const override { return "Drawing"; }
};

// 编辑状态实现
class EditingState : public SelectionState {
    // 实现编辑模式下的所有交互逻辑
};

// 多边形绘制状态实现
class PolygonDrawingState : public SelectionState {
    // 实现多边形绘制的特殊交互逻辑
};
```

### 职责分离与模块化

#### Selector类重构
**重构前**: 庞大的单一类，包含绘制、编辑、选择、渲染等所有功能
**重构后**: 专注于交互协调，委托具体功能给专门的类

```cpp
class Selector : public vtkInteractorStyleTrackballCamera {
private:
    // 核心组件
    vtkSmartPointer<vtkRenderer> renderer_;
    std::unique_ptr<PointCloudSelector> pointCloudSelector_;  // 点云选择
    std::unique_ptr<SelectionState> currentState_;           // 状态管理
    
    // 图形存储
    std::vector<std::unique_ptr<VectorShape>> vectorShapes_;
    
    // 撤销系统
    std::stack<std::unique_ptr<Command>> commandHistory_;

public:
    // 状态管理
    void SetState(std::unique_ptr<SelectionState> state);
    void EnableDrawingMode(bool enable);
    bool IsDrawingModeEnabled() const;
    
    // 委托给PointCloudSelector
    void ClearAllSelectedPoints();
    size_t GetSelectedPointCount() const;
    
    // 撤销系统
    void ExecuteCommand(std::unique_ptr<Command> command);
    void Undo();
};
```

#### VectorShape类增强
**新增功能**: 直接的点包含检测，消除Shape类依赖

```cpp
class VectorShape {
public:
    // 核心功能
    virtual bool containsPoint(double x, double y) const = 0;  // 新增：直接点包含检测
    virtual void draw(vtkRenderer* renderer, vtkActor2D* actor, ...) = 0;
    virtual bool hitTest(double x, double y, double tolerance = 8.0) const = 0;
    
    // 状态管理
    virtual std::string serialize() const = 0;
    virtual void deserialize(const std::string& data) = 0;
    
    // 向后兼容
    virtual ShapeData toShapeData() const = 0;  // 替代原有的toShape()
};
```

### 算法优化与性能提升

#### 遮挡检测算法
**原实现**: 简单距离比较
**优化后**: 基于深度缓冲的像素级检测

```cpp
std::vector<vtkIdType> PointCloudSelector::filterOccludedPoints(
    const std::vector<vtkIdType>& candidatePoints,
    const std::map<vtkIdType, std::pair<double, double>>& screenPositions,
    const std::map<vtkIdType, double>& distancesToCamera) {
    
    // 按距离排序（从近到远）
    std::vector<vtkIdType> sortedPoints = candidatePoints;
    std::sort(sortedPoints.begin(), sortedPoints.end(), 
              [&distancesToCamera](vtkIdType a, vtkIdType b) {
                  return distancesToCamera.at(a) < distancesToCamera.at(b);
              });
    
    // 基于深度缓冲的遮挡检测
    std::map<std::pair<int, int>, double> depthBuffer;
    std::vector<vtkIdType> visiblePoints;
    
    for (vtkIdType pointId : sortedPoints) {
        // 像素级深度检测逻辑
        if (!isOccluded(pointId, depthBuffer)) {
            visiblePoints.push_back(pointId);
            updateDepthBuffer(pointId, depthBuffer);
        }
    }
    
    return visiblePoints;
}
```

#### 点云选择流程优化
```cpp
std::vector<vtkIdType> PointCloudSelector::selectPointsByShapes(
    const std::vector<VectorShape*>& shapes) {
    
    // 1. 收集候选点（屏幕空间投影）
    std::vector<vtkIdType> candidatePoints = collectCandidatePoints(shapes);
    
    // 2. 遮挡检测（可选）
    std::vector<vtkIdType> visiblePoints;
    if (occlusionDetectionEnabled_) {
        visiblePoints = filterOccludedPoints(candidatePoints, screenPositions, distances);
    } else {
        visiblePoints = candidatePoints;
    }
    
    // 3. 累积选择
    addSelectedPoints(visiblePoints);
    
    return visiblePoints;
}
```

### 代码质量提升

#### 代码量对比
- **头文件**: 从509行减少到458行（减少10%）
- **源文件**: 从1976行重构为约2090行的结构化实现
- **类数量**: 移除1个复杂类（Shape），新增4个职责明确的类

#### 扩展性改进
1. **新增形状类型**: 只需继承VectorShape并实现核心方法
2. **新增交互模式**: 只需继承SelectionState并实现事件处理
3. **新增选择算法**: 只需扩展PointCloudSelector的方法

#### 维护性改善
1. **职责分离**: 每个类专注单一职责
2. **减少耦合**: 通过接口和委托减少类间依赖
3. **状态管理**: 清晰的状态转换，易于调试和扩展

### 设计模式应用

#### 1. 状态模式 (State Pattern)
```cpp
// 上下文类
class Selector {
    std::unique_ptr<SelectionState> currentState_;
public:
    void OnLeftButtonDown() {
        currentState_->handleMouseDown(this, x, y);
    }
};

// 状态类层次
SelectionState → DrawingState
              → EditingState  
              → PolygonDrawingState
```

#### 2. 命令模式 (Command Pattern)
```cpp
// 命令接口
class Command {
public:
    virtual void execute() = 0;
    virtual void undo() = 0;
};

// 具体命令
class StateChangeCommand : public Command;
class AddShapeCommand : public Command;
class DeleteShapeCommand : public Command;

// 调用者
class Selector {
    std::stack<std::unique_ptr<Command>> commandHistory_;
public:
    void ExecuteCommand(std::unique_ptr<Command> command);
    void Undo();
};
```

#### 3. 策略模式 (Strategy Pattern)
```cpp
// 策略接口
class VectorShape {
public:
    virtual bool containsPoint(double x, double y) const = 0;
};

// 具体策略
class VectorRectangle : public VectorShape;
class VectorCircle : public VectorShape;
class VectorPolygon : public VectorShape;

// 上下文
class PointCloudSelector {
    bool isPointInShapes(double x, double y, const std::vector<VectorShape*>& shapes) {
        for (auto& shape : shapes) {
            if (shape->containsPoint(x, y)) return true;
        }
        return false;
    }
};
```

### 重构效果总结

✅ **架构清晰**: 从混合架构转为纯面向对象架构
✅ **职责分离**: 每个类专注单一职责，便于维护
✅ **扩展性强**: 新增功能只需扩展相应的类或接口
✅ **代码复用**: 消除重复代码，提高代码质量
✅ **测试友好**: 模块化的设计便于单元测试
✅ **性能优化**: 算法优化和数据结构改进

这次重构为未来功能扩展和维护奠定了坚实的基础，同时保持了所有原有功能的完整性。

## 使用方法

### 基本操作
1. **启动应用**: 运行构建后的可执行文件
2. **导入点云**: 点击"导入PLY"按钮选择PLY文件
3. **3D交互**: 使用鼠标旋转、平移、缩放查看点云

### 矩形选择
1. **启用选择**: 点击"矩形选择"按钮
2. **绘制选择框**: 在点云上拖拽鼠标绘制矩形框
3. **查看结果**: 选中的点会以红色高亮显示

### 圆形选择
1. **启用选择**: 点击"圆形选择"按钮
2. **绘制选择区域**: 在点云上拖拽鼠标，以起终点为对角线的矩形内切圆为选择区域
3. **查看结果**: 选中的点会以红色高亮显示

### 多边形选择
1. **启用选择**: 点击"多边形选择"按钮
2. **绘制顶点**: 左键依次点击放置多边形顶点
3. **撤销顶点**: 按Backspace键撤销最后一个顶点
4. **完成选择**: 右键完成多边形绘制并执行选择
5. **查看结果**: 选中的点会以红色高亮显示

### 图形编辑操作

#### 选择图形
1. **退出绘制模式**: 确保不在绘制模式下（没有激活的绘制工具）
2. **点击图形**: 点击任意图形的边界进行选择
3. **查看状态**: 选中的图形变为红色，显示黄色控制点
4. **取消选择**: 点击空白区域取消选择

#### 编辑矩形
1. **选择矩形**: 点击矩形边界，图形变红色并显示8个控制点
2. **调整大小**: 拖拽角点或边中点调整矩形大小
3. **移动位置**: 拖拽矩形边界（非控制点区域）移动整个矩形
4. **完成编辑**: 点击空白区域或选择其他图形

#### 编辑圆形
1. **选择圆形**: 点击圆形边界，图形变红色并显示4个控制点
2. **调整半径**: 拖拽任意控制点调整圆的大小
3. **移动位置**: 拖拽圆形边界（非控制点区域）移动整个圆形
4. **完成编辑**: 点击空白区域或选择其他图形

#### 编辑多边形
1. **选择多边形**: 点击多边形边界，图形变红色并显示所有顶点控制点
2. **移动顶点**: 拖拽任意顶点控制点改变多边形形状
3. **移动整体**: 拖拽多边形边界（非顶点区域）移动整个多边形
4. **完成编辑**: 点击空白区域或选择其他图形

#### 删除图形
1. **选择图形**: 点击要删除的图形
2. **按删除键**: 按下Del键删除选中的图形
3. **撤销删除**: 可以使用Ctrl+Z撤销删除操作

#### 撤销操作
- **Ctrl+Z**: 撤销最近的一个操作
- **支持的操作**: 图形添加、删除、移动、形状编辑
- **历史限制**: 最多保存50个操作历史

### 高级功能
- **深度选择**: 通过工具栏复选框启用/禁用遮挡检测
- **累积选择**: 多次选择会累积高亮，不会覆盖之前的选择
- **清除选择**: 点击"清除选择"按钮恢复原始显示
- **撤销系统**: 支持Ctrl+Z撤销所有图形操作
- **快捷键操作**: Del键删除选中图形，Ctrl+Z撤销操作

## 故障排除

### 常见问题

1. **VTK路径错误**: 确保VTK_DIR路径正确指向您的VTK安装目录
2. **Qt路径错误**: 确保Qt5_DIR路径正确指向您的Qt安装目录
3. **编译器版本**: 确保使用与Qt和VTK兼容的编译器版本
4. **PLY文件格式**: 确保PLY文件格式正确，包含点坐标数据

### 调试信息
如果构建失败，请检查：
- CMake错误信息
- 编译器错误信息
- 路径配置是否正确
- VTK和Qt版本兼容性

## 快捷键参考

| 操作 | 快捷键 | 适用情况 |
|------|--------|----------|
| 撤销操作 | Ctrl+Z | 全局有效 |
| 删除图形 | Del | 选中图形时 |
| 撤销顶点 | Backspace | 多边形绘制时 |
| 取消选择 | 点击空白区域 | 图形选中时 |
| 完成多边形 | 右键 | 多边形绘制时 |

## 许可证

本项目仅供学习和研究使用。

## 点云裁剪应用

这是一个基于VTK和Qt的点云处理应用，支持PLY文件的导入和多种形状的选区功能。

### 主要功能

#### 1. 文件导入
- 支持PLY格式点云文件导入
- 自动根据Z轴高度生成颜色映射

#### 2. 画布绘制功能（重构后）
新的选区功能采用"画布"概念，支持多种形状的组合选择：

**绘制工具**：
- **矩形绘制**：拖拽鼠标绘制矩形选区
- **圆形绘制**：拖拽鼠标绘制圆形选区  
- **多边形绘制**：左键放置顶点，Backspace撤销上一个顶点，右键完成绘制

**画布操作**：
- **清空画布**：清除画布上的所有已绘制形状
- **确认选取**：根据画布上所有形状的并集进行实际的点云选取
- **清除选中点**：清除当前高亮显示的选中点

#### 3. 完整工作流程

##### 基础操作
1. **导入点云**：使用"导入PLY"加载点云文件
2. **绘制选区**：
   - 选择绘制工具（矩形/圆形/多边形）
   - 在3D视图中绘制多个形状
   - 所有形状都会保存在"画布"上，用绿色线条显示

##### 图形编辑（新功能）
3. **编辑图形**：
   - 退出绘制模式，点击图形进行选择（变红色）
   - 拖拽控制点调整形状大小或移动顶点
   - 拖拽图形边界移动整个图形
   - 使用Del键删除不需要的图形
   - 使用Ctrl+Z撤销错误操作

##### 完成选取
4. **执行选取**：点击"确认选取"按钮，根据画布上的所有形状执行点云选取
5. **管理选区**：
   - 使用"清空画布"重新开始绘制
   - 使用"清除选中点"清除选取结果

#### 4. 高级功能
- **深度选择**：启用后会过滤被遮挡的点，只选择前景可见的点
- **形状并集**：画布上的多个形状自动形成并集，点在任意形状内都会被选中
- **实时状态**：状态栏显示画布形状数量和选中点数量
- **画布锁定**：锁定画布可以禁用视图旋转和缩放，避免绘制时误操作
- **自动清理**：确认选取后画布会自动清空，准备下次绘制

### 编译和运行

```bash
# 编译
./build.bat

# 运行
# 编译完成后运行生成的可执行文件
```

### 系统要求
- Qt 5.12+
- VTK 8.2+
- CMake 3.16+
- Visual Studio 2019+ (Windows)

### 使用技巧

#### 绘制技巧
1. 可以组合使用不同类型的形状（矩形+圆形+多边形）
2. 绘制过程中，黄色线条表示正在绘制的形状，绿色线条表示已完成的形状
3. 多边形绘制时可以使用Backspace键撤销最后一个顶点
4. 启用深度选择可以避免选中被其他点遮挡的背景点
5. 在绘制复杂形状时，建议勾选"锁定画布"以避免意外的视图操作

#### 编辑技巧
6. **图形选择**: 只有在非绘制模式下才能选择和编辑图形
7. **控制点识别**: 黄色圆点是控制点，可以拖拽调整形状
8. **整体移动**: 点击图形边界（非控制点）可以移动整个图形
9. **快速删除**: 选中图形后按Del键快速删除
10. **安全操作**: 所有操作都可以用Ctrl+Z撤销，不用担心误操作
11. **状态指示**: 红色图形表示选中状态，绿色表示正常状态
12. **编辑精度**: 拖拽操作支持像素级精度，适合精细调整

#### 工作流程技巧
13. 确认选取后画布会自动清空，无需手动清理
14. 建议先完成所有图形的绘制和编辑，再执行"确认选取"
15. 复杂选区可以先绘制大致形状，再通过编辑功能精确调整 