#define MYTP_DRIVER_NAME "mytp_demo"

#define MYTP_PROTOCOL_A
//#define MYTP_PROTOCOL_B

#define LCM_X_RESOLUTION_MAX 1920
#define LCM_Y_RESOLUTION_MAX 1080
#define MYTP_FINGER_NUM_MAX 10
#define FINGER_OSR 64
#define ACTIVEPEN_OSR 260
#define ACTIVEPEN_PRESSURE_MAX 4096

#define PACKET_SIZE 67
#define MYTP_FINGERS_PKT 0x3F
#define MYTP_ACTIVEPEN_PKT 0x0D

struct mytp_fwinfo_data{
	u32 mytp_fw_mode;
	u32 mytp_fw_ver;
	u32 mytp_fw_id;
	u32 mytp_bcode_ver;
	u32 mytp_tx_num;
	u32 mytp_rx_num;
	u32 finger_x_resolution_max;
	u32 finger_y_resolution_max;
	u32 activepen_x_resolution_max;
	u32 activepen_y_resolution_max;
};

struct mytp_platform_data{
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 mytp_irq;
	struct i2c_client *client;
	struct input_dev *finger_input_dev;
	struct input_dev *activepen_input_dev;
	struct mytp_fwinfo_data fwinfo;
};

/* Debug levels */
#define NO_DEBUG 0
#define DEBUG_ERROR 1
#define DEBUG_MESSAGES 2
#define DEBUG_INFO 3

static int debug = DEBUG_INFO;
#define mytp_debug(level, ...) \
	do { \
		if (debug >= (level)) \
			printk("[mytp_drivers]:" __VA_ARGS__); \
		} while(0)
		