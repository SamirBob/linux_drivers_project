#define MYTP_DRIVER_NAME "mytp_demo"

/* Debug levels */
#define NO_DEBUG 0
#define DEBUG_ERROR 1
#define DEBUG_INFO 2

static int debug = DEBUG_INFO;
#define my_touch_debug(level, ...) \
	do { \
		if (debug >= (level)) \
			printk("[my_tp_drivers]:" __VA_ARGS__); \
		} while(0)