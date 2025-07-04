#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "pointcloudviewer.h"
#include "fileimporter.h"

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class QMenuBar;
class QStatusBar;
class QToolBar;
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void createActions();
    void createMenus();
    void createToolBars();
    void createStatusBar();
    void setupUI();
    void onImportPLY();
    void onFileSelected(const QString &fileName);
    void onFileSelectionCancelled();
    void onImportCompleted(bool success, const QString &message);
    void onRectangleSelection();
    void onCircleSelection();
    void onPolygonSelection();
    void onClearCanvas();
    void onConfirmSelection();
    void onClearSelection();
    void onOcclusionDetectionToggled(bool checked);
    void onViewLockToggled(bool checked);
    void onConvertRectangleToPolygon();
    void updateStatusInfo();

private:
    // UI组件
    PointCloudViewer *pointCloudViewer;
    FileImporter *fileImporter;
    QMenu *fileMenu;
    QMenu *helpMenu;
    QToolBar *fileToolBar;
    QAction *exitAct;
    QAction *aboutAct;
    QAction *importPLYAct;
    QAction *rectangleSelectionAct;
    QAction *circleSelectionAct;
    QAction *polygonSelectionAct;
    QAction *clearCanvasAct;
    QAction *confirmSelectionAct;
    QAction *clearSelectionAct;
    QAction *convertRectangleToPolygonAct;
};

#endif // MAINWINDOW_H 