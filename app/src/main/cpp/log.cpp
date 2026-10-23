#include "common.h"

#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace sphere {

namespace {
std::mutex gLogMutex;
FILE *gLogFile = nullptr;
}  // namespace

void setLogFile(const std::string &path) {
    std::lock_guard<std::mutex> lock(gLogMutex);
    if (gLogFile) fclose(gLogFile);
    gLogFile = path.empty() ? nullptr : fopen(path.c_str(), "w");
}

void logLine(int priority, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    __android_log_write(priority, "SphereStitcher", buf);
    std::lock_guard<std::mutex> lock(gLogMutex);
    if (gLogFile) {
        fprintf(gLogFile, "%s%s\n", priority >= ANDROID_LOG_WARN ? "W " : "", buf);
        fflush(gLogFile);
    }
}

}  // namespace sphere
