/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#ifndef __KERNEL_CLOG_H__
#define __KERNEL_CLOG_H__

typedef enum logger_LogPriority {
    LOGGER_LOG_UNKNOWN = 0,
    LOGGER_LOG_DEFAULT,
    LOGGER_LOG_VERBOSE,
    LOGGER_LOG_DEBUG,
    LOGGER_LOG_INFO,
    LOGGER_LOG_WARN,
    LOGGER_LOG_ERROR,
    LOGGER_LOG_FATAL,
    LOGGER_LOG_SILENT,
} logger_LogPriority;

/*
 * This is the local tag used for the following simplified
 * logging macros.  You can change this preprocessor definition
 * before using the other macros to change the tag.
 */
#ifndef LOG_TAG
#define LOG_TAG "kernel"
#endif

/*
 * Simplified macro to send a verbose confidential log message.
 */
#ifndef CLOG
#define CLOG(tag, ...) ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_VERBOSE, tag, __VA_ARGS__))
#endif


/*
 * Simplified macro to send a verbose confidential log message using the current LOG_TAG.
 */
#ifndef CLOGV
#define CLOGV(...) ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_VERBOSE, LOG_TAG, __VA_ARGS__))
#endif

#define CONDITION(cond)     (__builtin_expect((cond)!=0, 0))

#ifndef CLOGV_IF
#define CLOGV_IF(cond, ...) \
    ( (CONDITION(cond)) \
    ? ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)) \
    : (void)0 )
#endif

/*
 * Simplified macro to send a debug confidential log message using the current LOG_TAG.
 */
#ifndef CLOGD
#define CLOGD(...) ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_DEBUG, LOG_TAG, __VA_ARGS__))
#endif

#ifndef CLOGD_IF
#define CLOGD_IF(cond, ...) \
    ( (CONDITION(cond)) \
    ? ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_DEBUG, LOG_TAG, __VA_ARGS__)) \
    : (void)0 )
#endif

/*
 * Simplified macro to send an info confidential log message using the current LOG_TAG.
 */
#ifndef CLOGI
#define CLOGI(...) ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_INFO, LOG_TAG, __VA_ARGS__))
#endif

#ifndef CLOGI_IF
#define CLOGI_IF(cond, ...) \
    ( (CONDITION(cond)) \
    ? ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_INFO, LOG_TAG, __VA_ARGS__)) \
    : (void)0 )
#endif

/*
 * Simplified macro to send a warning confidential log message using the current LOG_TAG.
 */
#ifndef CLOGW
#define CLOGW(...) ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_WARN, LOG_TAG, __VA_ARGS__))
#endif

#ifndef CLOGW_IF
#define CLOGW_IF(cond, ...) \
    ( (CONDITION(cond)) \
    ? ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_WARN, LOG_TAG, __VA_ARGS__)) \
    : (void)0 )
#endif

/*
 * Simplified macro to send an error confidential log message using the current LOG_TAG.
 */
#ifndef CLOGE
#define CLOGE(...) ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_ERROR, LOG_TAG, __VA_ARGS__))
#endif

#ifndef CLOGE_IF
#define CLOGE_IF(cond, ...) \
    ( (CONDITION(cond)) \
    ? ((void)__logger_write_to_log(LOG_ID_CNFD, LOGGER_LOG_ERROR, LOG_TAG, __VA_ARGS__)) \
    : (void)0 )
#endif


/*
 * ===========================================================================
 *
 * The stuff in the rest of this file should not be used directly.
 */

typedef enum {
    LOG_ID_MAIN = 0,
    LOG_ID_RADIO = 1,
    LOG_ID_EVENTS = 2,
    LOG_ID_SYSTEM = 3,
    LOG_ID_CNFD = 4,

    LOG_ID_MAX
} log_id_t;

/*
 * Send a simple string to the log.
 */
int logger_write_to_log(int bufID, int prio, const char *tag, const char *fmt, ...);
#define __logger_write_to_log(bufID, prio, tag, ...)		\
	logger_write_to_log(bufID, prio, tag, __VA_ARGS__)
#endif

