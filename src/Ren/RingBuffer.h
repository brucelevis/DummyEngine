#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <atomic>

namespace Ren {
    template <class T>
    class RingBuffer {
        T* buf_;
        std::atomic_int head_, tail_;
        int             size_;

        int next(int cur) const {
            return (cur + 1) % size_;
        }
    public:
        explicit RingBuffer(int size) : size_(size) {
            head_ = 0;
            tail_ = 0;
            buf_ = new T[size];
        }
        ~RingBuffer() {
            delete[] buf_;
        }
        RingBuffer(const RingBuffer&) = delete;
        RingBuffer& operator=(const RingBuffer&) = delete;

        bool Empty() const {
            const int tail = tail_.load(std::memory_order_relaxed);
            return (tail == head_.load(std::memory_order_acquire));
        }

        bool Full() const {
            const int head = head_.load(std::memory_order_relaxed);
            const int next_head = next(head);
            return next_head == tail_.load(std::memory_order_acquire);
        }

        int Capacity() const {
            return size_;
        }

        bool Push(const T& item) {
            const int head = head_.load(std::memory_order_relaxed);
            const int next_head = next(head);
            if (next_head == tail_.load(std::memory_order_acquire)) {
                return false;
            }
            buf_[head] = item;
            head_.store(next_head, std::memory_order_release);

            return true;
        }

        bool Push(T&& item) {
            const int head = head_.load(std::memory_order_relaxed);
            const int next_head = next(head);
            if (next_head == tail_.load(std::memory_order_acquire)) {
                return false;
            }
            buf_[head] = std::move(item);
            head_.store(next_head, std::memory_order_release);

            return true;
        }

        bool Pop(T& item) {
            const int tail = tail_.load(std::memory_order_relaxed);
            if (tail == head_.load(std::memory_order_acquire)) {
                return false;
            }
            item = std::move(buf_[tail]);
            tail_.store(next(tail), std::memory_order_release);

            return true;
        }
    };
}

#endif // RING_BUFFER_H