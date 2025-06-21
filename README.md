# 点云处理应用

这是一个基于VTK 8.2.0和Qt 5.12.9的点云可视化与处理应用，支持PLY文件导入、3D可视化、矩形框选和点云高亮功能。

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
│   └── rectangleselector.h # 矩形选择器头文件
└── src/                   # 源文件目录
    ├── main.cpp           # 主程序入口
    ├── mainwindow.cpp     # 主窗口实现
    ├── mainwindow.ui      # Qt Designer UI文件
    ├── pointcloudviewer.cpp # 点云查看器实现
    ├── fileimporter.cpp   # 文件导入器实现
    └── rectangleselector.cpp # 矩形选择器实现
```

## 环境要求

- **VTK**: 8.2.0 (安装在 `D:\code\vtk8.2.0\VTK-8.2.0`)
- **Qt**: 5.12.9 (安装在 `C:\Qt\Qt5.12.9\5.12.9\msvc2017_64`)
- **编译器**: Visual Studio 2022 或更高版本
- **CMake**: 3.14 或更高版本

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
- **矩形框选**: 支持鼠标拖拽矩形框选点云
- **点云高亮**: 选中的点以红色高亮显示
- **交互式操作**: 支持鼠标旋转、平移、缩放3D场景
- **Qt界面**: 现代化的Qt用户界面，包含菜单、工具栏和状态栏

## 核心功能

### 点云导入与显示
- 通过菜单或工具栏导入PLY文件
- 自动根据Z坐标生成渐变色显示
- 支持点云统计信息显示（点数、面片数、Z范围）

### 矩形选择功能
- 点击"矩形选择"按钮启用选择模式
- 鼠标拖拽绘制选择框
- 实时显示选择框，不随3D场景旋转
- 支持多次选择，选中点累积高亮

### 选择管理
- 选中点以红色高亮显示
- 支持清除所有选中点，恢复原始颜色
- 选中状态持久保存，直到手动清除

## 数据流路径

### 选择过程
1. `PerformPointSelection()` → 检测哪些点在选择框内
2. 将新选中的点ID添加到 `selectedPointIds` 向量中
3. `selectedPointIds.insert(selectedPointIds.end(), newSelectedPointIds.begin(), newSelectedPointIds.end());`

### 高亮显示
1. `HighlightSelectedPoints(selectedPointIds)` → 根据ID列表修改点云颜色
2. 直接修改 `originalPointData` 中对应点的颜色为红色
3. 未选中的点保持原始渐变色

### 清除选择
1. `ClearAllSelectedPoints()` → 清空 `selectedPointIds` 并恢复原始颜色
2. 从 `originalColorBackup` 恢复所有点的原始颜色

## 技术架构

### 主要类说明
- **MainWindow**: 主窗口，负责UI和信号槽连接
- **PointCloudViewer**: 点云查看器，集成VTK渲染窗口
- **FileImporter**: 文件导入器，处理PLY文件选择
- **RectangleSelector**: 矩形选择器，实现点云选择功能

### 数据存储
- 选中点存储在 `RectangleSelector::selectedPointIds` 中（点ID列表）
- 原始颜色备份在 `RectangleSelector::originalColorBackup` 中
- 点云数据通过 `vtkPolyData` 管理

## 使用方法

1. **启动应用**: 运行构建后的可执行文件
2. **导入点云**: 点击"导入PLY"按钮选择PLY文件
3. **启用选择**: 点击"矩形选择"按钮启用选择模式
4. **选择点云**: 在点云上拖拽鼠标绘制选择框
5. **查看结果**: 选中的点会以红色高亮显示
6. **清除选择**: 点击"清除选择"按钮恢复原始显示

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

## 许可证

本项目仅供学习和研究使用。 