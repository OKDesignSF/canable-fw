#ifndef __LOCK_H
#define __LOCK_H

#include <stdbool.h>

bool lock_enter(void);
void lock_exit(bool enter_state);

#endif // __LOCK_H
