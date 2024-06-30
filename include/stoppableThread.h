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
    bool isTerminated();
    void detach();
    void stopThread();
    void join();
    bool* isJoined();
    void setJoined(bool val);

private:
    bool stopRequested;
    bool* joined = nullptr;
    bool terminated = false;
    std::thread t;
};

#endif // STOPPABLETHREAD_H
