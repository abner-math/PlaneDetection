#ifndef WORKER_H
#define WORKER_H

#include <QThread>

class Worker : public QObject
{
    Q_OBJECT
public:
    Worker();

    virtual ~Worker();

    void run();

    void stop();

    bool isRunning() const
    {
        return !mStopped;
    }

    double timeElapsedInSec() const
    {
        return mTimeElapsed / 1e9;
    }

signals:
    void workerStart();
    void workerProgress(float progress);
    void workerFinish();
    void workerStatus(const QString &status);

protected:
    virtual void actions() = 0;

private slots:
    void onRun();

private:
    QThread mThread;
    double mTimeElapsed;
    bool mStopped;

};

#endif // WORKER_H
