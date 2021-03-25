//
// Created by dyq on 2020/11/19.
//


#ifndef ACE_COMMON_BLOCKING_QUEUE_H_
#define ACE_COMMON_BLOCKING_QUEUE_H_

#include <cstddef>
#include <deque>
#include <memory>

#include "absl/synchronization/mutex.h"
#include "ace/common/port.h"
#include "ace/common/time.h"
#include "glog/logging.h"

namespace ace {
namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
template <typename T>
class BlockingQueue
{
public:
    static constexpr size_t kInfiniteQueueSize = 0;

    // Constructs a blocking queue with infinite queue size.
    BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}

    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;

    // Constructs a blocking queue with a size of 'queue_size'.
    explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

    // Pushes a value onto the queue. Blocks if the queue is full.
    void Push(T t)
    {
        const auto predicate = [this]()
        {
            return QueueNotFullCondition();
        };

        absl::MutexLock lock(&mutex_);
        mutex_.Await(absl::Condition(&predicate));
        deque_.push_back(std::move(t));
    }

    // Like push, but returns false if 'timeout' is reached.
    bool PushWithTimeout(T t, const common::Duration timeout)
    {
        const auto predicate = [this]()
        {
            return QueueNotFullCondition();
        };

        absl::MutexLock lock(&mutex_);
        if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate), absl::FromChrono(timeout)))
        {
            return false;
        }
        deque_.push_back(std::move(t));
        return true;
    }

    // Pops the next value from the queue. Blocks until a value is available.
    T Pop()
    {
        const auto predicate = [this]()
        {
            return !QueueEmptyCondition();
        };

        absl::MutexLock lock(&mutex_);
        mutex_.Await(absl::Condition(&predicate));

        T t = std::move(deque_.front());
        deque_.pop_front();
        return t;
    }

    T End()
    {
        const auto predicate = [this]()
        {
            return !QueueEmptyCondition();
        };

        absl::MutexLock lock(&mutex_);
        mutex_.Await(absl::Condition(&predicate));

        T t = std::move(deque_.back());
        return t;
    }


    // Like Pop, but can timeout. Returns nullptr in this case.
    T PopWithTimeout(const common::Duration timeout)
    {
        const auto predicate = [this]()
        {
            return !QueueEmptyCondition();
        };

        absl::MutexLock lock(&mutex_);
        if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate), absl::FromChrono(timeout)))
        {
            return nullptr;
        }

        T t = std::move(deque_.front());
        deque_.pop_front();
        return t;
    }

    // Like Peek, but can timeout. Returns nullptr in this case.
    template <typename R>
    R* PeekWithTimeout(const common::Duration timeout)
    {
        const auto predicate = [this]()
        {
            return !QueueEmptyCondition();
        };

        absl::MutexLock lock(&mutex_);
        if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate), absl::FromChrono(timeout))) {
          return nullptr;
        }
        return deque_.front().get();
    }

    // Returns the next value in the queue or nullptr if the queue is empty.
    // Maintains ownership. This assumes a member function get() that returns
    // a pointer to the given type R.
    template <typename R>
    const R* Peek()
    {
        absl::MutexLock lock(&mutex_);
        if (deque_.empty())
        {
            return nullptr;
        }

        return deque_.front().get();
    }

    // Returns the number of items currently in the queue.
    size_t Size()
    {
        absl::MutexLock lock(&mutex_);
        return deque_.size();
    }

    // Blocks until the queue is empty.
    void WaitUntilEmpty()
    {
        const auto predicate = [this]()
        {
            return QueueEmptyCondition();
        };

        absl::MutexLock lock(&mutex_);
        mutex_.Await(absl::Condition(&predicate));
    }

private:
    // Returns true if the queue is empty.
    bool QueueEmptyCondition()
    {
        return deque_.empty();
    }

    // Returns true iff the queue is not full.
    bool QueueNotFullCondition()
    {
        return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
    }

    absl::Mutex mutex_;
    const size_t queue_size_;
    std::deque<T> deque_;
};

}  // namespace common
}  // namespace ace

#endif  // ACE_COMMON_BLOCKING_QUEUE_H_
