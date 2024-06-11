#include "stoppableThread.h"

StoppableThread::StoppableThread() : stopRequested(false) {}

StoppableThread::~StoppableThread() {
    stopRequested = true;
}


bool* StoppableThread::startThread(std::function<void()> func) {
    t = std::thread([this, func = std::move(func)] {
        func();
        joined = true;
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

void StoppableThread::join() {
    joined = false;
    t.join();
    joined = true;
}

bool StoppableThread::isJoined() {
    return joined;
}

void StoppableThread::setJoined(bool val) {
    joined = val;
}