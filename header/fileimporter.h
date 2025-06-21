#ifndef FILEIMPORTER_H
#define FILEIMPORTER_H

#include <QObject>
#include <QString>
#include <QFileDialog>

class FileImporter : public QObject
{
    Q_OBJECT

public:
    explicit FileImporter(QObject *parent = nullptr);
    ~FileImporter();

    // 选择PLY文件
    QString selectPLYFile(QWidget *parent = nullptr);

signals:
    // 文件选择完成信号
    void fileSelected(const QString &fileName);
    void fileSelectionCancelled();

private:
    // 文件对话框过滤器
    static const QString PLY_FILTER;
};

#endif // FILEIMPORTER_H 