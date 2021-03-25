//
// Created by dyq on 2020/10/28.
//


#ifndef ACE_COMMON_TASK_H_
#define ACE_COMMON_TASK_H_

#include <set>

#include "absl/synchronization/mutex.h"
#include "glog/logging.h"
#include "thread_pool.h"

namespace ace {
namespace common {

class ThreadPoolInterface;

class Task
{
public:
    friend class ThreadPoolInterface;

    using WorkItem = std::function<void()>;
    enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

    Task() = default;
    ~Task();

    State GetState();

    // State must be 'NEW'.
    void SetWorkItem(const WorkItem& work_item);

    // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
    // assumed completed.
    void AddDependency(std::weak_ptr<Task> dependency);

private:
    // Allowed in all states.
    void AddDependentTask(Task* dependent_task);

    // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
    void Execute();

    // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
    void SetThreadPool(ThreadPoolInterface* thread_pool);

    // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
    // 'DEPENDENCIES_COMPLETED'.
    void OnDependenyCompleted();

    WorkItem work_item_;
    ThreadPoolInterface* thread_pool_to_notify_ = nullptr;
    State state_ = NEW;
    unsigned int uncompleted_dependencies_ = 0;
    std::set<Task*> dependent_tasks_;

    absl::Mutex mutex_;
};

}  // namespace common
}  // namespace ace

#endif  // ACE_COMMON_TASK_H_
