/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */


#include "logcapture.h"

#include <linux/hrtimer.h>
#include <linux/kmod.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>

#ifndef KEY_SHARP
#define KEY_SHARP       253
#endif

#define DUMPLOG_TIMER      30
#define LOG_KEY_NONE       0x0
#define LOG_KEY_SHARP      0x02
#define LOG_KEY_KPASTERISK 0x01
#define LOG_KEY_ON         (LOG_KEY_KPASTERISK | LOG_KEY_SHARP)

#ifdef CONFIG_LOGCAPTURE_DEBUG
#define LOGCAPTURE_PRINT(fmt, ...) printk(KERN_DEBUG "logcapture: " fmt, ##__VA_ARGS__)
#else
#define LOGCAPTURE_PRINT(fmt, ...) do {} while (0)
#endif

static struct hrtimer logcapture_timer;
static int logcapture_status = LOG_KEY_NONE;

#define EVENT_NONE   0
#define EVENT_HAPPEN 1
wait_queue_head_t logcapture_wait_queue;

static atomic_t event_status;

static inline int get_event_status(void)
{
	return atomic_read(&event_status);
}

static inline void set_event_status(int status)
{
	atomic_set(&event_status, status);
}


void check_logcapture(struct input_dev* dev,
                               unsigned int code, int value)
{
	if (value == 0) {
		if (code == KEY_SHARP) {
			LOGCAPTURE_PRINT("logcapture status=0x%x dev=%p\n",
				logcapture_status, dev);
			logcapture_status &= ~LOG_KEY_SHARP;
			LOGCAPTURE_PRINT("logcapture timer cancel\n");
			hrtimer_cancel(&logcapture_timer);
			LOGCAPTURE_PRINT("logcapture after status=0x%x\n",
				logcapture_status);
		} else if (code == KEY_KPASTERISK) {
			LOGCAPTURE_PRINT("logcapture status=0x%x dev=%p\n",
				logcapture_status, dev);
			logcapture_status &= ~LOG_KEY_KPASTERISK;
			LOGCAPTURE_PRINT("logcapture timer cancel\n");
			hrtimer_cancel(&logcapture_timer);
			LOGCAPTURE_PRINT("logcapture after status=0x%x\n",
				logcapture_status);
		}
	} else {
		if (code == KEY_SHARP) {
			LOGCAPTURE_PRINT("logcapture status=0x%x dev=%p\n",
				logcapture_status, dev);
			logcapture_status |= LOG_KEY_SHARP;
			if (logcapture_status == LOG_KEY_ON) {
				LOGCAPTURE_PRINT("logcapture timer start\n");
				hrtimer_start(&logcapture_timer,
				ktime_set(DUMPLOG_TIMER,0),
				HRTIMER_MODE_REL);
			}
			LOGCAPTURE_PRINT("logcapture after status=0x%x\n",
				logcapture_status);
		} else if (code == KEY_KPASTERISK) {
			LOGCAPTURE_PRINT("logcapture status=0x%x dev=%p \n",
				logcapture_status, dev);
			logcapture_status |= LOG_KEY_KPASTERISK;
			if (logcapture_status == LOG_KEY_ON) {
				LOGCAPTURE_PRINT("logcapture timer start\n");
				hrtimer_start(&(logcapture_timer),
					ktime_set(DUMPLOG_TIMER,0),
					HRTIMER_MODE_REL);
			}
			LOGCAPTURE_PRINT("logcapture after status=0x%x\n",
				logcapture_status);
		}
	}
}


static int execute_logcapture(void)
{
	set_event_status(EVENT_HAPPEN);
	wake_up_interruptible(&logcapture_wait_queue);
	return 0;
}

static enum hrtimer_restart logcapture_timer_func(struct hrtimer *timer)
{
	LOGCAPTURE_PRINT("logcapture_timer_func start\n");
	execute_logcapture();
	return HRTIMER_NORESTART;
}


static ssize_t logcapture_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	const u8 happen = 1;

	if (EVENT_NONE == get_event_status()) {
		if (filp->f_flags & O_NONBLOCK) {
			LOGCAPTURE_PRINT("return nonblocking is specified");
			return -EAGAIN;
		}

		ret = wait_event_interruptible(logcapture_wait_queue, EVENT_NONE != get_event_status());
		if (ret) {
			LOGCAPTURE_PRINT("logcapture wait ret = %d", ret);
			return ret;
		}
	}

	LOGCAPTURE_PRINT("event_status = %d", get_event_status());
	set_event_status(EVENT_NONE);
	count = sizeof(happen);
	if (copy_to_user(buf, &happen, count)) {
		LOGCAPTURE_PRINT("invalid buffer address");
		return -EFAULT;
	}
	return count;
}

static unsigned int logcapture_poll(struct file *file, poll_table *wait)
{
	if (EVENT_NONE == get_event_status())
	{
		poll_wait(file, &logcapture_wait_queue, wait);
	}

	LOGCAPTURE_PRINT("exit poll event_status = %d", get_event_status());
	if (EVENT_NONE == get_event_status()) {
		return 0;
	}
	return POLLIN | POLLRDNORM;
}

static int logcapture_open(struct inode *inode, struct file *file)
{
	LOGCAPTURE_PRINT("open");
	return 0;
}

static int logcapture_release(struct inode *inode, struct file *file)
{
	LOGCAPTURE_PRINT("release");
	return 0;
}

static const struct file_operations logcapture_fops = {
	.owner = THIS_MODULE,
	.read = logcapture_read,
	.poll = logcapture_poll,
	.open = logcapture_open,
	.release = logcapture_release,
};

static struct miscdevice logcapture_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "logcapture",
	.fops = &logcapture_fops,
};


/* call from input_init */
void __init logcapture_init(void)
{
	int ret;
	LOGCAPTURE_PRINT("logcapture_init\n");
	hrtimer_init(&logcapture_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	logcapture_timer.function = logcapture_timer_func;

	atomic_set(&event_status, EVENT_NONE);
	init_waitqueue_head(&logcapture_wait_queue);

	ret = misc_register(&logcapture_misc);
	if(unlikely(ret)) {
		pr_err("failed to register device (MISC_DYNAMIC_MINOR)\n");
		return;
	}
}

/* call from input_exit */
void __exit logcapture_exit(void)
{
	LOGCAPTURE_PRINT("logcapture_exit\n");

	misc_deregister(&logcapture_misc);

	hrtimer_cancel(&logcapture_timer);
}
