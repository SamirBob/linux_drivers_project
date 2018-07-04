#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>

#include "my_tp_drivers.h"

static int mytp_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	my_touch_debug(DEBUG_INFO,"mytp probe start!");
	my_touch_debug(DEBUG_INFO,"mytp probe end!");
	return 0;
}

static int mytp_remove(struct i2c_client *client)
{
	my_touch_debug(DEBUG_INFO,"mytp remove start!");
	my_touch_debug(DEBUG_INFO,"mytp remove end!");
	return 0;
}

static const struct i2c_device_id mytp_id[] = {
	{MYTP_DRIVER_NAME,0},
	{},
};

static const struct of_device_id mytp_match_table[] = {
	{.compatible = "firelly,mytp_demo",},
	{},
};

static struct i2c_driver mytp_driver = {
	.probe = mytp_probe,
	.remove = mytp_remove,
	.id_table = mytp_id,
	.driver = {
		.name = MYTP_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mytp_match_table,
	},
};

static __init int mytp_init(void)
{
	int ret ;
	my_touch_debug(DEBUG_INFO,"mytp init start!");
	ret = i2c_add_driver(&mytp_driver);
	if(ret != 0){
		my_touch_debug(DEBUG_ERROR,"mytp driver init failed!");
	}
	my_touch_debug(DEBUG_INFO,"mytp init end!");
	return ret;
}

static __exit void mytp_exit(void)
{
	my_touch_debug(DEBUG_INFO,"mytp exit start!");
	i2c_del_driver(&mytp_driver);
	my_touch_debug(DEBUG_INFO,"mytp exit end!");	
}

module_init(mytp_init);
module_exit(mytp_exit);

MODULE_AUTHOR("samir <samir@163.com.cn>");
MODULE_DESCRIPTION("TP Driver Demo");
MODULE_LICENSE("GPL");