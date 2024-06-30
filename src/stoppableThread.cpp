#include <iostream>
#include "stoppableThread.h"

StoppableThread::StoppableThread() : stopRequested(false) {}

StoppableThread::~StoppableThread() {
    stopRequested = true;
}


bool* StoppableThread::startThread(std::function<void()> func) {
    terminated = false;
    t = std::thread([this, func = std::move(func)] {
        func();
        terminated = true;
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

bool StoppableThread::isTerminated() {
    return terminated;
}

void StoppableThread::join() {
    if (joined == nullptr || !*joined) {
        joined = new bool(false);
        t.join();
        *joined = true;
        std::cout << "Thread joined" << std::endl;
    }
}

bool* StoppableThread::isJoined() {
    return joined;
}

void StoppableThread::setJoined(bool val) {
    if (joined != nullptr){
        *joined = val;
    }else{
        joined = new bool(val);
    }
}