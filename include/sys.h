#ifndef SYS_H_
#define SYS_H_

// glibc includes
#include <time.h>

int sys_set_time(time_t time);

void sys_reset();

#endif // SYS_H_
