#include "ProcessManager.h"

#include <QTimer>

namespace tracking_app {

ProcessManager::ProcessManager(QObject* parent) : QObject(parent) {
    proc_ = new QProcess(this);
    proc_->setProcessChannelMode(QProcess::SeparateChannels);
    connect(proc_, &QProcess::readyReadStandardOutput,
            this, &ProcessManager::onReadyReadStdout);
    connect(proc_, &QProcess::readyReadStandardError,
            this, &ProcessManager::onReadyReadStderr);
    connect(proc_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, &ProcessManager::onFinished);
    connect(proc_, &QProcess::errorOccurred,
            this, &ProcessManager::onErrorOccurred);
}

ProcessManager::~ProcessManager() {
    if (isRunning()) {
        proc_->kill();
        proc_->waitForFinished(2000);
    }
}

void ProcessManager::start(const QString& exe, const QStringList& args, const QString& workingDir) {
    if (isRunning()) stop();
    if (!workingDir.isEmpty()) proc_->setWorkingDirectory(workingDir);
    stdout_buf_.clear();
    stderr_buf_.clear();
    proc_->start(exe, args);
    if (proc_->waitForStarted(3000)) {
        emit started(proc_->processId());
    } else {
        emit error(QStringLiteral("启动失败: %1\n参数: %2")
                       .arg(proc_->errorString(), args.join(' ')));
    }
}

void ProcessManager::stop() {
    if (!isRunning()) return;
    proc_->terminate();
    if (!proc_->waitForFinished(5000)) {
        proc_->kill();
        proc_->waitForFinished(2000);
    }
}

bool ProcessManager::isRunning() const {
    return proc_->state() != QProcess::NotRunning;
}

qint64 ProcessManager::pid() const {
    return proc_->processId();
}

void ProcessManager::onReadyReadStdout() {
    stdout_buf_ += proc_->readAllStandardOutput();
    emitLines(stdout_buf_);
}

void ProcessManager::onReadyReadStderr() {
    stderr_buf_ += proc_->readAllStandardError();
    emitLines(stderr_buf_);
}

void ProcessManager::emitLines(const QByteArray& /*chunk*/) {
    // 选择 stdout 还是 stderr 由调用栈决定 (调用方传入对应 buf 引用更复杂,
    // 这里简化: 共用 stdout_buf_, 所有 stderr 进 stderr_buf_, 各自抽取行)
    // 实际逻辑: 提取 stdout_buf_ 和 stderr_buf_ 中的完整行
    auto extract = [this](QByteArray& buf) {
        int idx;
        while ((idx = buf.indexOf('\n')) >= 0) {
            QString line = QString::fromUtf8(buf.left(idx)).trimmed();
            if (!line.isEmpty()) emit output(line);
            buf.remove(0, idx + 1);
        }
    };
    extract(stdout_buf_);
    extract(stderr_buf_);
}

void ProcessManager::onFinished(int code, QProcess::ExitStatus status) {
    // flush 剩余 buf
    if (!stdout_buf_.isEmpty()) {
        emit output(QString::fromUtf8(stdout_buf_).trimmed());
        stdout_buf_.clear();
    }
    if (!stderr_buf_.isEmpty()) {
        emit output(QString::fromUtf8(stderr_buf_).trimmed());
        stderr_buf_.clear();
    }
    emit stopped(code, status);
}

void ProcessManager::onErrorOccurred(QProcess::ProcessError err) {
    if (err == QProcess::FailedToStart) {
        emit error(QStringLiteral("无法启动二进制: %1").arg(proc_->errorString()));
    }
}

}  // namespace tracking_app
