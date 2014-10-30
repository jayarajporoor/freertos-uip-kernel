#include <klog.h>
#include <stdarg.h>

static int log_level;

void klog_init()
{
	log_level = KLOG_ERROR|KLOG_INFO;
}

void klog_set_log_level(int level)
{
	log_level = level;
}

void klog_info_on()
{
	log_level |= KLOG_INFO;
}

void klog_info_off()
{
	log_level &= ~KLOG_INFO;
}

void klog_error(const char *format, ...)
{
	va_list ap;
	char buf[128];
	
	if(log_level > KLOG_ERROR) return;
	buf[0] = 'E'; buf[1] = 'R'; buf[2] = 'R'; buf[3] = ':';	
	va_start(ap, format);
	//vsnprintf(&buf[4], sizeof(buf), format, ap);
	//TODO output buf to log device
	va_end(ap);
}
void klog_warning(const char *format, ...)
{
	va_list ap;
	char buf[128];
	
	if(log_level > KLOG_WARNING) return;
	
	buf[0] = 'W'; buf[1] = 'R'; buf[2] = 'N'; buf[3] = ':';
	va_start(ap, format);
	//vsnprintf(&buf[4], sizeof(buf), format, ap);
	//TODO output buf to log device
	va_end(ap);
}
void klog_debug(const char *format, ...)
{
	va_list ap;
	char buf[128];
	
	if(log_level > KLOG_DEBUG) return;
	
	buf[0] = 'D'; buf[1] = 'B'; buf[2] = 'G'; buf[3] = ':';
	va_start(ap, format);
	//vsnprintf(&buf[4], sizeof(buf), format, ap);
	//TODO output buf to log device
	va_end(ap);	
}

void klog_info(const char *format, ...)
{
	va_list ap;
	char buf[128];
	
	if(!(log_level&KLOG_INFO)) return;
	
	buf[0] = 'I'; buf[1] = 'N'; buf[2] = 'F'; buf[3] = ':';
	va_start(ap, format);
	//vsnprintf(&buf[4], sizeof(buf), format, ap);
	//TODO output buf to log device
	va_end(ap);
	
}
