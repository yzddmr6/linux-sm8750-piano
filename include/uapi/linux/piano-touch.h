/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_PIANO_TOUCH_H
#define _UAPI_LINUX_PIANO_TOUCH_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* Set on the exclusive stream fd. 0 disables scanning; 1..3 select the
 * controller's M80P, N83P or P81C model. Requests survive firmware reloads
 * and are deferred while the panel is off or the device is suspended.
 * Closing the stream discards the request. Errors affect only the pen.
 */
#define PIANO_TOUCH_SET_PEN _IOW('P', 0x20, __u32)

/* Reading arms POLLPRI notifications for later panel/system-sleep changes.
 * A successful read acknowledges the pending notification. */
#define PIANO_TOUCH_GET_STATE _IOR('P', 0x21, __u32)
#define PIANO_TOUCH_STATE_ACTIVE 1U

#endif
