//
// Created by dyq on 2020/9/9.
//

#ifndef QF_CR_FUSION_LOG_H
#define QF_CR_FUSION_LOG_H

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <string>

namespace ace {
namespace common {


#define LOG_VERSION "0.1.0"

typedef void (*log_LockFn)(void *udata, int lock);

enum { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };


#define LogTrace(...)  ace::common::Log::Write(ace::common::LOG_TRACE, __FILENAME__, __LINE__, __VA_ARGS__)
#define LogDebug(...)  ace::common::Log::Write(ace::common::LOG_DEBUG, __FILENAME__, __LINE__, __VA_ARGS__)
#define LogInfo(...)   ace::common::Log::Write(ace::common::LOG_INFO,  __FILENAME__, __LINE__, __VA_ARGS__)
#define LogWarn(...)   ace::common::Log::Write(ace::common::LOG_WARN,  __FILENAME__, __LINE__, __VA_ARGS__)
#define LogError(...)  ace::common::Log::Write(ace::common::LOG_ERROR, __FILENAME__, __LINE__, __VA_ARGS__)
#define LogFatal(...)  ace::common::Log::Write(ace::common::LOG_FATAL, __FILENAME__, __LINE__, __VA_ARGS__)
#define LogInitFile(filename)  ace::common::Log::SetFp(filename)


#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

class Log
{
public:
    Log();

    static void Write(int level, const char *file, int line, const char *fmt, ...);

    static void SetUdata(void *udata);

    static void SetLock(log_LockFn fn);

    static void SetFp(FILE *fp);

    static void SetLevel(int level);

    static void SetQuiet(int enable);

    FILE *fp;
};

}  // namespace common
}  // namespace ace


#endif //QF_CR_FUSION_LOG_H
