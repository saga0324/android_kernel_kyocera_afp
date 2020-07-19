#include <linux/ql/os.h>
#include <linux/ql/qltrace.h>

#ifndef TRACE_ENABLED_MODULES
#define TRACE_ENABLED_MODULES	(~0)
#endif

#ifndef TRACE_LEVEL_ENABLED
#define TRACE_LEVEL_ENABLED	(~0)
#endif

#define QL_TRACE_PRINT_MAX (512)

static unsigned int TraceEnabledModuleBitmask = TRACE_ENABLED_MODULES, TraceLevelsEnabled = TRACE_LEVEL_ENABLED;

QL_Status QL_Trace(unsigned int module, unsigned int tracelevel, char *header, const char *function, unsigned int linenum, char *format, ...)
{
	int size;
	char tracebuf[QL_TRACE_PRINT_MAX] = {0};
	va_list va;
	
	/* Check whether tracing for the module specified is enabled or not.
	 * Not enabling debugging for a particular module is not an error condition
	 * so we return QL_STATUS_OK here.
	 */
	if (!((module & TraceEnabledModuleBitmask) && (tracelevel & TraceLevelsEnabled)))
		return QL_STATUS_OK;
	
	va_start(va, format);
	size = vsnprintf(tracebuf, sizeof(tracebuf) - 1, format, va);
	va_end(va);

	if (size < 0)
		return QL_STATUS_ERROR;

	/* print to console using normal printf - any alternate functions can be used
	 * here to redirect the trace messages
	 */
	printk("%s:[%s@%d]:%s", header, function, linenum, tracebuf);

	return QL_STATUS_OK;
}