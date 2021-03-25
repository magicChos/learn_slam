//
// Created by dyq on 2020/10/28.
//


#ifndef ACE_COMMON_THREAD_POOL_H_
#define ACE_COMMON_THREAD_POOL_H_

#include <deque>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "ace/common/task.h"

namespace ace {
namespace common {

class Task;

class ThreadPoolInterface
{
public:
    ThreadPoolInterface() {}
    virtual ~ThreadPoolInterface() {}
    virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

protected:
    void Execute(Task* task);
    void SetThreadPool(Task* task);

private:
    friend class Task;

    virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

// A fixed number of threads working on tasks. Adding a task does not block.
// Tasks may be added whether or not their dependencies are completed.
// When all dependencies of a task are completed, it is queued up for execution
// in a background thread. The queue must be empty before calling the
// destructor. The thread pool will then wait for the currently executing work
// items to finish and then destroy the threads.
class ThreadPool : public ThreadPoolInterface
{
public:
    explicit ThreadPool(int num_threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // When the returned weak pointer is expired, 'task' has certainly completed,
    // so dependants no longer need to add it as a dependency.
    std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) override;

private:
    void DoWork();

    void NotifyDependenciesCompleted(Task* task) override;

    absl::Mutex mutex_;
    bool running_ = true;
    std::vector<std::thread> pool_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    absl::flat_hash_map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
};

}  // namespace common
}  // namespace ace

#endif  // ACE_COMMON_THREAD_POOL_H_
