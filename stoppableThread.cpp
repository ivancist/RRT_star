#include "stoppableThread.h"

StoppableThread::StoppableThread() : stopRequested(false) {}

StoppableThread::~StoppableThread() {
    stopRequested = true;
}


bool* StoppableThread::startThread(std::function<void()> func) {
    t = std::thread([this, func = std::move(func)] {
        func();
    });
    return &stopRequested;
}

void StoppableThread::detach() {
    t.detach();
}

void StoppableThread::stopThread() {
    stopRequested = true;
}

bool StoppableThread::isStopRequested() {
    return stopRequested;
}