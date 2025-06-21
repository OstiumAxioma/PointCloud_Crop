#include "fileimporter.h"
#include <QFileDialog>

const QString FileImporter::PLY_FILTER = "PLY文件 (*.ply);;所有文件 (*.*)";

FileImporter::FileImporter(QObject *parent)
    : QObject(parent)
{
}

FileImporter::~FileImporter()
{
}

QString FileImporter::selectPLYFile(QWidget *parent)
{
    QString fileName = QFileDialog::getOpenFileName(
        parent,
        "选择PLY文件",
        "",
        PLY_FILTER
    );

    if (fileName.isEmpty()) {
        emit fileSelectionCancelled();
    } else {
        emit fileSelected(fileName);
    }

    return fileName;
} 