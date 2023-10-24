#pragma once

#include <cppQueue.h>

/**
 * FIFO Queue
 */
template<typename T>
class FifoQueue {
private:
    cppQueue queue;
    unsigned long topInProgressTs = 0;
public:
    FifoQueue(int maxSize) :queue(sizeof(T), maxSize, FIFO) { }

    void setTopInProgress() {
        topInProgressTs = millis();
    }

    void push(const T& val) {
        queue.push(&val);
    }

    T stopTopInProgress() {
        T ret;
        queue.pop(&ret);
        topInProgressTs = 0;
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
        return topInProgressTs != 0;
    }

    unsigned long getTopTimestamp() const {
        return topInProgressTs;
    }
};
