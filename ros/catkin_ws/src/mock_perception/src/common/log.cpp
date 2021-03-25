//
// Created by dyq on 2020/9/9.
//


#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <ctime>

#include "ace/common/log.h"

namespace ace {
namespace common {


Log log;
#define LOG_USE_COLOR

static struct {
    void *udata;
    log_LockFn lock;
    FILE *fp;
    int level;
    int quiet;
} L;

// "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
static const char *level_names[] = {
    "[ T ]", "[ D ]", "[ I ]", "[ W ]", "[ E ]", "[ F ]"
};

#ifdef LOG_USE_COLOR
static const char *level_colors[] = {
	"\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"
};
#endif

static void lock(void)
{
    if (L.lock) {
        L.lock(L.udata, 1);
    }
}


static void unlock(void)
{
    if (L.lock) {
        L.lock(L.udata, 0);
    }
}


Log::Log()
{
    time_t now = time(0);
    std::string filename = "./system.log";
   
    fp = fopen(filename.c_str(), "w+");
    LogInitFile(fp);
}


void Log::Write(int level, const char *file, int line, const char *fmt, ...)
{
    if (level < L.level) {
        return;
    }

    /* Acquire lock */
    lock();

    /* Get current time */
    time_t t = time(NULL);
    struct tm *lt = localtime(&t);

    /* Log to stderr */
#if 0//zjwang remove print log to std
    if (!L.quiet) {
        va_list args;
        char buf[32];
        buf[strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", lt)] = '\0';
#ifdef LOG_USE_COLOR
        fprintf(stderr, "\x1b[0m[%s]%s %s %s:%d ",
			buf, level_colors[level], level_names[level], file, line);
#else
        fprintf(stderr, "[%s %s] %s:%d: ", buf, level_names[level], file, line);
#endif
        va_start(args, fmt);
        vfprintf(stderr, fmt, args);
        va_end(args);
        fprintf(stderr, "\n");
        fflush(stderr);
    }
#endif

    /* Log to file */
    if (L.fp) {
        va_list args;
        char buf[32];
        buf[strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", lt)] = '\0';
        fprintf(L.fp, "[%s %-5s %s]:%d: ", buf, level_names[level], file, line);
        va_start(args, fmt);
        vfprintf(L.fp, fmt, args);
        va_end(args);
        fprintf(L.fp, "\n");
        fflush(L.fp);
    }

    /* Release lock */
    unlock();
}




void Log::SetFp(FILE *fp)
{
    L.fp = fp;
}

void Log::SetLevel(int level)
{
    L.level = level;
}

void Log::SetLock(ace::common::log_LockFn fn)
{
    L.lock = fn;
}

void Log::SetUdata(void *udata)
{
    L.udata = udata;
}

void Log::SetQuiet(int enable)
{
    L.quiet = enable ? 1 : 0;
}


}  // namespace common
}  // namespace ace