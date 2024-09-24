#ifndef THREAD_3DMOUSE_H
#define THREAD_3DMOUSE_H

#include <QString>
#include <QThread>
#include <QMutex>

class Thread3DMouse : public QThread
{
    Q_OBJECT

public:
    explicit Thread3DMouse(QObject *parent = nullptr);
    ~Thread3DMouse() override;

    void  startThread();
    void run() Q_DECL_OVERRIDE;
    void close();

signals:
    void error(const QString &s);
    void timeout(const QString &s);
    void motionEvent(int x,int y,int z,int rx,int ry,int rz);

private:
    QMutex mutex;
    bool quit;
    QMutex mtx_value;

};

#endif // THREAD_3DMOUSE_H
