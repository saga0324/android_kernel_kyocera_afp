/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

#ifndef _QL_TRACE_H_
#define _QL_TRACE_H_
#include "os.h"
#include "qlspi.h"

/* These macros can be used to disable particular trace level per file.
 * QL_TRACE_XXXX_ENABLE	- can be undefined in a particular file to disable XXXX Trace level for that particular file alone.
 * QL_TRACE_XXXX_ENABLE	- can be undefined in a particular file to disable trace for XXXX module for that particular file alone.
 */
#define QL_TRACE_ERROR_ENABLE
#define QL_TRACE_WARN_ENABLE
#define QL_TRACE_DEBUG_ENABLE

#define QL_TRACE_SPI_ENABLE
#define QL_TRACE_MCU_ENABLE


/**********************************/

#ifndef BIT
#define BIT(_x_) (1 << (_x_))
#endif

/* Debug Levels */
#define QL_DEBUG_TRACE_ERROR	BIT(0)
#define QL_DEBUG_TRACE_WARN		BIT(1)
#define QL_DEBUG_TRACE_DEBUG	BIT(2)
#define QL_DEBUG_TRACE_INFO	BIT(3)

/* Modules */
#define QL_MODULE_SPI		BIT(0)
#define QL_MODULE_MCU		BIT(1)
#define QL_MODULE_TEST		BIT(2)


/* Enable Tracing for below given modules .
 * This will be used for runtime check to emit trace messages from the particular modules
 */
#define TRACE_ENABLED_MODULES	(QL_MODULE_SPI | QL_MODULE_MCU | QL_MODULE_TEST)

/* Enable Trace levels as given below for all modules mentioned above (TRACE_ENABLED_MODULES).
 * This will be used for runtime check to emit trace messages of particular trace levels.
 */
#define TRACE_LEVEL_ENABLED	(QL_DEBUG_TRACE_ERROR | QL_DEBUG_TRACE_WARN | QL_DEBUG_TRACE_INFO)
//#define TRACE_LEVEL_ENABLED	(QL_DEBUG_TRACE_ERROR | QL_DEBUG_TRACE_WARN | QL_DEBUG_TRACE_DEBUG)

#define QL_Trace QLSPI_Trace

QL_Status QL_Trace(unsigned int module, unsigned int tracelevel, char *header, const char *function, unsigned int linenum, char *format, ...);

#define QL_TRACE(_MODULE_, _TRACE_LEVEL_, _TRACE_HEADER_, _FMT_, ...)	\
	QL_Trace(_MODULE_, _TRACE_LEVEL_, _TRACE_HEADER_, ___FUNCTION__, __LINE__, _FMT_, ##__VA_ARGS__)

#ifdef QL_TRACE_ERROR_ENABLE
#define QL_TRACE_ERROR(_MODULE_, _FMT_, ...)	\
	pr_err("["#_MODULE_"][ERROR] ql %s %d"_FMT_, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define QL_TRACE_ERROR(_MODULE_, _FMT_, ...)
#endif

#ifdef QL_TRACE_WARN_ENABLE
#define QL_TRACE_WARN(_MODULE_, _FMT_, ...)	\
	pr_notice("["#_MODULE_"][WARN] ql %s %d"_FMT_, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define QL_TRACE_WARN(_MODULE_, _FMT_, ...)
#endif

#ifdef QL_TRACE_DEBUG_ENABLE
#define QL_TRACE_DEBUG(_MODULE_, _FMT_, ...)	\
	pr_debug("[" #_MODULE_ "][DEBUG] ql %s %d"_FMT_, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define QL_TRACE_DEBUG(_MODULE_, _FMT_, ...)
#endif

#ifdef QL_TRACE_INFO_ENABLE
#define QL_TRACE_INFO(_MODULE_, _FMT_, ...)	\
	pr_info("[" #_MODULE_ "][INFO] ql %s %d"_FMT_, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define QL_TRACE_INFO(_MODULE_, _FMT_, ...)
#endif

#ifdef QL_TRACE_SPI_ENABLE
#define QL_TRACE_SPI_DEBUG(_FMT_, ...)	QL_TRACE_DEBUG(QL_MODULE_SPI, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_SPI_ERROR(_FMT_, ...)	QL_TRACE_ERROR(QL_MODULE_SPI, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_SPI_WARN(_FMT_, ...)	QL_TRACE_WARN(QL_MODULE_SPI, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_SPI_INFO(_FMT_, ...)	QL_TRACE_INFO(QL_MODULE_SPI, _FMT_, ##__VA_ARGS__)
#else
#define QL_TRACE_SPI_DEBUG(_FMT_, ...)
#define QL_TRACE_SPI_ERROR(_FMT_, ...)	QL_TRACE_ERROR(QL_MODULE_SPI, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_SPI_WARN(_FMT_, ...)
#define QL_TRACE_SPI_INFO(_FMT_, ...)
#endif


#ifdef QL_TRACE_MCU_ENABLE
#define QL_TRACE_MCU_DEBUG(_FMT_, ...)	QL_TRACE_DEBUG(QL_MODULE_MCU, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_MCU_ERROR(_FMT_, ...)	QL_TRACE_ERROR(QL_MODULE_MCU, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_MCU_WARN(_FMT_, ...)	QL_TRACE_WARN(QL_MODULE_MCU, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_MCU_INFO(_FMT_, ...)	QL_TRACE_INFO(QL_MODULE_MCU, _FMT_, ##__VA_ARGS__)
#else
#define QL_TRACE_MCU_DEBUG(_FMT_, ...)
#define QL_TRACE_MCU_ERROR(_FMT_, ...)	QL_TRACE_ERROR(QL_MODULE_MCU, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_MCU_WARN(_FMT_, ...)
#define QL_TRACE_MCU_INFO(_FMT_, ...)
#endif

#ifdef QL_TRACE_TEST_ENABLE
#define QL_TRACE_TEST_DEBUG(_FMT_, ...)	QL_TRACE_DEBUG(QL_MODULE_TEST, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_TEST_ERROR(_FMT_, ...)	QL_TRACE_ERROR(QL_MODULE_TEST, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_TEST_WARN(_FMT_, ...)	QL_TRACE_WARN(QL_MODULE_TEST, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_TEST_INFO(_FMT_, ...)	QL_TRACE_INFO(QL_MODULE_TEST, _FMT_, ##__VA_ARGS__)
#else
#define QL_TRACE_TEST_DEBUG(_FMT_, ...)
#define QL_TRACE_TEST_ERROR(_FMT_, ...)	QL_TRACE_ERROR(QL_MODULE_TEST, _FMT_, ##__VA_ARGS__)
#define QL_TRACE_TEST_WARN(_FMT_, ...)
#define QL_TRACE_TEST_INFO(_FMT_, ...)
#endif

#endif	/* _QL_TRACE_H_ */