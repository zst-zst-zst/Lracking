#ifndef CONTROL_PANEL_PROCESS_MANAGER_H
#define CONTROL_PANEL_PROCESS_MANAGER_H

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>

namespace tracking_app {

// 启动/停止子进程 (tracking 或 tracking_main), 转发 stdout/stderr 信号
class ProcessManager : public QObject {
    Q_OBJECT

public:
    explicit ProcessManager(QObject* parent = nullptr);
    ~ProcessManager() override;

    // 启动二进制. 已运行时先停后起.
    void start(const QString& exe, const QStringList& args, const QString& workingDir = {});

    // 停止: 先 SIGTERM, 5s 内不退强杀
    void stop();

    bool isRunning() const;
    qint64 pid() const;

signals:
    void started(qint64 pid);
    void stopped(int exitCode, QProcess::ExitStatus status);
    void output(const QString& line);   // 单行 stdout/stderr
    void error(const QString& msg);     // 启动失败等

private slots:
    void onReadyReadStdout();
    void onReadyReadStderr();
    void onFinished(int code, QProcess::ExitStatus status);
    void onErrorOccurred(QProcess::ProcessError err);

private:
    void emitLines(const QByteArray& chunk);

    QProcess* proc_ = nullptr;
    QByteArray stdout_buf_;
    QByteArray stderr_buf_;
};

}  // namespace tracking_app

#endif  // CONTROL_PANEL_PROCESS_MANAGER_H
