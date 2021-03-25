//
// Created by dyq on 2020/11/12.
//


#include "ace/common/thread_pool.h"
#include "absl/memory/memory.h"
#include "gtest/gtest.h"

#include <iostream>
#include <vector>


namespace ace {
namespace common {
namespace {

class Receiver
{
public:
    void Receive(int number)
    {
        absl::MutexLock locker(&mutex_);
        received_numbers_.push_back(number);
    }

    void WaitForNumberSequence(const std::vector<int>& expected_numbers)
    {
        const auto predicate =
        [this, &expected_numbers]()  {
            return (received_numbers_.size() >= expected_numbers.size());
        };
        absl::MutexLock locker(&mutex_);
        mutex_.Await(absl::Condition(&predicate));
        EXPECT_EQ(expected_numbers, received_numbers_);
    }

    absl::Mutex mutex_;
    std::vector<int> received_numbers_ GUARDED_BY(mutex_);
};

TEST(ThreadPoolTest, RunTask)
{
    ThreadPool pool(1);
    Receiver receiver;
    auto task = absl::make_unique<Task>();
    task->SetWorkItem([&receiver]() { receiver.Receive(1); });
    pool.Schedule(std::move(task));
    receiver.WaitForNumberSequence({1});
}


TEST(ThreadPoolTest, ManyTasks)
{
    for (int a = 0; a < 5; ++a)
    {
        ThreadPool pool(3);
        Receiver receiver;
        int kNumTasks = 10;
        for (int i = 0; i < kNumTasks; ++i)
        {
            auto task = absl::make_unique<Task>();
            task->SetWorkItem([&receiver]() { receiver.Receive(1); });
            pool.Schedule(std::move(task));
        }
        receiver.WaitForNumberSequence(std::vector<int>(kNumTasks, 1));
    }
}

TEST(ThreadPoolTest, RunWithDependency)
{
    ThreadPool pool(2);
    Receiver receiver;

    auto task_2 = absl::make_unique<Task>();
    task_2->SetWorkItem([&receiver]() { receiver.Receive(2); });

    auto task_1 = absl::make_unique<Task>();
    task_1->SetWorkItem([&receiver]() { receiver.Receive(1); });

    auto weak_task_1 = pool.Schedule(std::move(task_1));
    task_2->AddDependency(weak_task_1);

    pool.Schedule(std::move(task_2));
    receiver.WaitForNumberSequence({1, 2});
}

class ReceiveData
{
public:
    void HandleSize(int s)
    {
        size = s;
        LOG(INFO) << "Handle size: " << size ;
    }

    void HandleProcess(int p)
    {
        process = p;
        LOG(INFO)  << "Handle process: " << process ;
    }


private:
    double size;
    int process;
};


TEST(ThreadPoolTest, testReceiveData)
{
    ThreadPool pool(2);
    ReceiveData data;

    auto task_1 = absl::make_unique<Task>();
    task_1->SetWorkItem([&data]() { data.HandleSize(100); });
    pool.Schedule(std::move(task_1));

    auto task_2 = absl::make_unique<Task>();
    task_2->SetWorkItem([&data]() { data.HandleProcess(200); });
    pool.Schedule(std::move(task_2));

}


}
}
}