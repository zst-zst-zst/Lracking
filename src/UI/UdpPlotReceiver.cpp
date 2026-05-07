#include "UdpPlotReceiver.h"

#include <QHostAddress>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QNetworkDatagram>
#include <QUdpSocket>

namespace tracking_app {

UdpPlotReceiver::UdpPlotReceiver(quint16 port, QObject* parent)
    : QObject(parent), port_(port) {
    sock_ = new QUdpSocket(this);
    // ShareAddress 让外部 PlotJuggler 仍可同时监听同端口 (双方都需 ShareAddress).
    bound_ = sock_->bind(QHostAddress::AnyIPv4, port_,
                         QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    if (!bound_) {
        last_err_ = sock_->errorString();
        emit boundChanged(false, QString("UDP %1 \u7ed1\u5b9a\u5931\u8d25: %2").arg(port_).arg(last_err_));
        return;
    }
    connect(sock_, &QUdpSocket::readyRead, this, &UdpPlotReceiver::onReadyRead);
    emit boundChanged(true, QString("UDP %1 \u76d1\u542c\u4e2d (\u4e0e PlotJuggler \u5171\u4eab)").arg(port_));
}

void UdpPlotReceiver::onReadyRead() {
    while (sock_->hasPendingDatagrams()) {
        QNetworkDatagram dg = sock_->receiveDatagram();
        QByteArray data = dg.data();
        QJsonParseError err{};
        QJsonDocument doc = QJsonDocument::fromJson(data, &err);
        if (doc.isNull() || !doc.isObject()) continue;
        QJsonObject obj = doc.object();
        QHash<QString, double> kv;
        kv.reserve(obj.size());
        for (auto it = obj.constBegin(); it != obj.constEnd(); ++it) {
            const QJsonValue& v = it.value();
            if (v.isDouble())      kv.insert(it.key(), v.toDouble());
            else if (v.isBool())   kv.insert(it.key(), v.toBool() ? 1.0 : 0.0);
            // \u5b57\u7b26\u4e32 / null / array / object \u8df3\u8fc7
        }
        if (!kv.isEmpty()) emit samples(kv);
    }
}

}  // namespace tracking_app
