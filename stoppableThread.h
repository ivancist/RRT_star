#ifndef STOPPABLETHREAD_H
#define STOPPABLETHREAD_H

#include <thread>
#include <atomic>
#include <functional>

class StoppableThread {
public:
    StoppableThread();
    ~StoppableThread();
    bool* startThread(std::function<void()> func);
    bool isStopRequested();
    void detach();
    void stopThread();

private:
    bool stopRequested;
    std::thread t;
};

#endif // STOPPABLETHREAD_H
