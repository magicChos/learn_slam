//
// Created by dyq on 2020/11/12.
//

#ifndef ACE_COMMON_SINGTON_H
#define ACE_COMMON_SINGTON_H

#include <mutex>
#include <memory>

namespace ace {
namespace common {

template<typename T>
class Singleton
{
public:
    // Get global singleton
    template<typename ...Args>
    static std::shared_ptr <T> GetInstance(Args &&... args)
    {
        if (!sington_)
        {
            std::lock_guard <std::mutex> gLock(mutex_);
            if (nullptr == sington_)
            {
                sington_ = std::make_shared<T>(std::forward<Args>(args)...);
            }
        }
        return sington_;
    }

    // Active destruct singleton objects do not need to be actively destructed except for special requirements
    static void DestroyInstance()
    {
        if (sington_)
        {
            sington_.reset();
            sington_ = nullptr;
        }
    }

private:
    explicit Singleton();

    Singleton(const Singleton &) = delete;

    Singleton &operator=(const Singleton &) = delete;

    ~Singleton();

private:
    static std::shared_ptr <T> sington_;
    static std::mutex mutex_;
};

template<typename T>
std::shared_ptr <T> Singleton<T>::sington_ = nullptr;

template<typename T>
std::mutex Singleton<T>::mutex_;

} // namespace common
} // namespace ace

#endif //ACE_COMMON_SINGTON_H
