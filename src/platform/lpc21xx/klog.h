#ifndef KLOG_H_
#define KLOG_H_

void klog_init();
void klog_error(const char *format, ...);
void klog_warning(const char *format, ...);
void klog_debug(const char *format, ...);

void klog_set_log_level(int level);
void klog_info_on();
void klog_info_off();

void klog_info(const char *format, ...);

#define KLOG_ERROR 		0
#define KLOG_WARNING 	1
#define KLOG_DEBUG		2
#define KLOG_INFO		4

#endif /*KLOG_H_*/
