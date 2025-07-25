# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PointCloud_Crop is a C++ application for point cloud visualization and selection using VTK and Qt. It provides CAD-style vector graphics editing capabilities for selecting regions in 3D point clouds.

## Build and Run Commands

```batch
# Build the project (Windows)
build.bat

# The executable will be generated at:
build\Exe\Release\VTK_Qt_Project.exe
```

## Development Environment

- **IDE**: Visual Studio 2022
- **C++ Standard**: C++11
- **Qt Version**: 5.12.9
- **VTK Version**: 8.2.0
- **Build System**: CMake

## Architecture Overview

The codebase follows a modular architecture with clear separation of concerns:

### Core Components

1. **PointCloudSelector** (selector.h/cpp)
   - Central class managing point cloud selection logic
   - Handles occlusion detection using depth buffer
   - Manages cumulative selection across multiple operations

2. **State Management Pattern**
   - `SelectionState` (abstract base): Defines state interface
   - `DrawingState`: Handles shape drawing interactions
   - `EditingState`: Manages shape editing operations
   - `NormalState`: Default interaction state

3. **Vector Shape System**
   - `VectorShape` (abstract base): Common shape interface
   - `RectangleShape`: Rectangle selection implementation
   - `CircleShape`: Circle selection implementation  
   - `PolygonShape`: Polygon selection implementation

4. **Command Pattern for Undo**
   - `DeleteCommand`: Handles shape deletion with undo support
   - Integrated with Qt's undo/redo system

### Key Design Patterns

- **State Pattern**: Different interaction modes (drawing, editing, normal)
- **Command Pattern**: Undo/redo functionality
- **Strategy Pattern**: Different shape types with common interface
- **Observer Pattern**: Qt signals/slots for UI updates

### Critical Implementation Details

1. **Occlusion Handling**: Uses VTK's depth buffer (vtkWindowToImageFilter) to determine visible points
2. **Coordinate Systems**: Transforms between world, view, and display coordinates
3. **Selection Logic**: Points inside shapes are checked against depth buffer for visibility
4. **Cumulative Selection**: Multiple selections accumulate using std::set to avoid duplicates

## Common Development Tasks

### Adding New Shape Types
1. Create new class inheriting from `VectorShape`
2. Implement required virtual methods (draw, contains, toPolygon)
3. Add shape creation logic in `DrawingState`
4. Update shape type enumeration

### Modifying Selection Logic
- Core selection logic is in `PointCloudSelector::processSelection()`
- Visibility checking uses `isPointVisible()` with depth buffer
- Point transformation logic is in `transformPointsToView()`

### UI Integration
- Main window logic in `mainwindow.cpp`
- VTK rendering handled by `QVTKOpenGLWidget`
- Shape rendering uses VTK actors and mappers

## Important Notes

- The project has undergone significant refactoring from v1.0 to v3.0 (see README.md)
- Selection colors: Pink (selected), Yellow (boundary), Green (editing handles)
- All coordinate transformations must account for VTK's coordinate system
- Depth buffer resolution affects occlusion detection accuracy