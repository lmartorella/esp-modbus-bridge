#pragma once

#include <cppQueue.h>

/**
 * FIFO Queue
 */
template<typename T>
class FifoQueue {
private:
    cppQueue queue;
    unsigned long lastTimestamp = 0;
    enum {
        Idle,
        InProgress
    } headState = Idle;
public:
    FifoQueue(int maxSize) 
        :queue(sizeof(T), maxSize, FIFO) { }

    void setHeadInProgress() {
        lastTimestamp = millis();
        headState = InProgress;
    }

    void push(const T& val) {
        queue.push(&val);
    }

    T dequeue() {
        T ret;
        queue.pop(&ret);
        headState = Idle;
        return ret;
    }

    T peek() const {
        T ret;
        (const_cast<cppQueue*>(&queue))->peek(&ret);
        return ret;
    }

    bool isEmpty() const {
        return (const_cast<cppQueue*>(&queue))->isEmpty();
    }

    bool isFull() const {
        return (const_cast<cppQueue*>(&queue))->isFull();
    }

    bool inProgress() const {
        return headState == InProgress;
    }

    unsigned long getHeadTimestamp() const {
        return lastTimestamp;
    }
};
