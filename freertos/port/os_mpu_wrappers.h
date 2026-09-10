/*
 * This port has no MPU wrappers -- every task runs privileged, so there's
 * no "unprivileged API call needs a wrapper" concept at all. os_portable.h
 * includes this file unconditionally, so it still needs to exist and
 * provide the two macros os_tasks.c/os_queue.c/os_timer.c/
 * os_event_groups.c decorate their privileged data/functions with.
 */

#ifndef MPU_WRAPPERS_H
#define MPU_WRAPPERS_H

#define PRIVILEGED_FUNCTION
#define PRIVILEGED_DATA

#endif /* MPU_WRAPPERS_H */
