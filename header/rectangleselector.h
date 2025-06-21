#ifndef RECTANGLESELECTOR_H
#define RECTANGLESELECTOR_H

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
#include <QObject>
#include <functional>

class RectangleSelector : public vtkInteractorStyleTrackballCamera
{
public:
    static RectangleSelector* New();
    vtkTypeMacro(RectangleSelector, vtkInteractorStyleTrackballCamera);

    // 设置渲染器和点云数据
    void SetRenderer(vtkRenderer* renderer);
    void SetPointCloudData(vtkPolyData* pointData);
    
    // 启用/禁用矩形选择模式
    void EnableRectangleSelection(bool enable);
    bool IsRectangleSelectionEnabled() const { return rectangleSelectionEnabled; }

    // 清除选择
    void ClearSelection();
    
    // 设置光标回调函数
    void SetCursorCallback(std::function<void(Qt::CursorShape)> callback) {
        cursorCallback = callback;
    }

protected:
    RectangleSelector();
    ~RectangleSelector();

    // 鼠标事件处理
    virtual void OnLeftButtonDown() override;
    virtual void OnLeftButtonUp() override;
    virtual void OnMouseMove() override;

private:
    // 绘制选择矩形
    void DrawSelectionRectangle();
    void ClearSelectionRectangle();
    
    // 执行点云选择
    void PerformPointSelection();
    
    // 高亮选中的点
    void HighlightSelectedPoints(const std::vector<vtkIdType>& selectedPointIds);

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
    
    // 选择框的四个角点
    vtkSmartPointer<vtkPoints> rectanglePoints;
    vtkSmartPointer<vtkPolyData> rectanglePolyData;
    vtkSmartPointer<vtkActor> rectangleActor;
    vtkSmartPointer<vtkPolyDataMapper> rectangleMapper;

    std::function<void(Qt::CursorShape)> cursorCallback;
};

#endif // RECTANGLESELECTOR_H 