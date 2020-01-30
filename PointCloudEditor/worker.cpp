#include "worker.h"

#include <QElapsedTimer>

Worker::Worker()
    : mStopped(true)
{
    moveToThread(&mThread);
    connect(&mThread, SIGNAL(started()), this, SLOT(onRun()));
}

Worker::~Worker()
{
    if (mThread.isRunning())
        mThread.terminate();
}

void Worker::run()
{
    if (!mThread.isRunning())
        mThread.start();
}

void Worker::stop()
{
    if (mThread.isRunning())
    {
        mStopped = true;
    }
}

void Worker::onRun()
{
    mStopped = false;
    QElapsedTimer timer;
    timer.start();
    emit workerStart();
    actions();
    mTimeElapsed = timer.nsecsElapsed();
    mThread.terminate();
    emit workerFinish();
    mStopped = true;
}
