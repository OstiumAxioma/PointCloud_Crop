#ifndef POINTCLOUDVIEWER_H
#define POINTCLOUDVIEWER_H

// VTK模块初始化
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)

#include <QVTKOpenGLWidget.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPLYReader.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkElevationFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include "selector.h"

class PointCloudViewer : public QVTKOpenGLWidget
{
    Q_OBJECT

public:
    explicit PointCloudViewer(QWidget *parent = nullptr);
    ~PointCloudViewer();

    // 初始化VTK组件
    bool initializeVTK();
    
    // 导入PLY文件
    bool importPLYFile(const QString &fileName);
    
    // 获取点云信息
    int getPointCount() const;
    int getCellCount() const;
    double getZRangeMin() const;
    double getZRangeMax() const;
    
    // 清除当前显示
    void clearDisplay();
    
    // 矩形选择功能
    void enableRectangleSelection(bool enable);
    void enableCircleSelection(bool enable);
    void enablePolygonSelection(bool enable);
    void enableOcclusionDetection(bool enable);
    void clearAllSelectedPoints();
    
    // 画布功能
    void clearCanvas();
    void confirmSelection();
    size_t getCanvasShapeCount() const;
    size_t getSelectedPointCount() const;
    
    // 视图锁定功能
    void enableViewLock(bool enable);

signals:
    // 导入完成信号
    void importCompleted(bool success, const QString &message);

private:
    // VTK组件
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkPLYReader> plyReader;
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter;
    vtkSmartPointer<vtkElevationFilter> elevationFilter;
    
    // 矩形选择器
    vtkSmartPointer<Selector> rectangleSelector;
    
    // 点云信息
    int pointCount;
    int cellCount;
    double zRangeMin;
    double zRangeMax;
};

#endif // POINTCLOUDVIEWER_H 