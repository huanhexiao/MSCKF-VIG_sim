/*
** navlog.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/include
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Dec 17 下午3:13:11 2018 little fang
** Last update Thu Jul 10 下午9:12:22 2019 little fang
*/

#ifndef NAVLOG_H_
#define NAVLOG_H_
#include <string>

#define GLOGOUTPUT

#ifdef GLOGOUTPUT
#include <glog/logging.h>
#endif // GLOGOUTPUT

namespace utiltool
{

inline void LogInit(const char *argv, const std::string &path, int loglevel = 0)
{
#ifdef GLOGOUTPUT
  FLAGS_stderrthreshold = loglevel;
  google::InitGoogleLogging((const char *)argv);
  google::SetLogDestination(0, path.c_str());
#else
  FLAGS_stderrthreshold = google::FATAL;
  google::InitGoogleLogging((const char *)argv);
  google::SetLogDestination(google::FATAL, path.c_str());
#endif
}

// #ifdef GLOGOUTPUT
// #define navinfolog(format, ...)                                                                                                            \
//   {                                                                                                                                        \
//     char log[4096] = {0};                                                                                                                  \
//     sprintf(log, format, ##__VA_ARGS__);                                                                                                   \
//     LOG(INFO) << log ;                                                                                                         \
//   }
// #define navwarnlog(format, ...)                                                                                                         \
//   {                                                                                                                                        \
//     char log[4096] = {0};                                                                                                                  \
//     sprintf(log, format, ##__VA_ARGS__);                                                                                                   \
//     LOG(WARNING) << log ;                                                                                                      \
//   }
// #define naverrorlog(format, ...)                                                                                                           \
//   {                                                                                                                                        \
//     char log[4096] = {0};                                                                                                                  \
//     sprintf(log, format, ##__VA_ARGS__);                                                                                                   \
//     LOG(ERROR) << log ;                                                                                                        \
//   }
// #define navfatallog(format, ...)                                                                                                           \
//   {                                                                                                                                        \
//     char log[4096] = {0};                                                                                                                  \
//     sprintf(log, format, ##__VA_ARGS__);                                                                                                   \
//     LOG(FATAL) << log ;                                                                                                        \
//   }
// #else
// #define navinfolog(format, ...)
// #define navwarnlog(format, ...)
// #define naverrorlog(format, ...)
// #define navfatallog(format, ...)
// #endif

} // namespace untiltool


#endif /* !NAVLOG_H_ */
