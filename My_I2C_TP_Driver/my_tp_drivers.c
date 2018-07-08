#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include "my_tp_drivers.h"

static int mytp_parse_dts(struct device *dev,struct mytp_platform_data *pdata)
{
	int rc =0;
	struct device_node *np = dev->of_node;
	mytp_debug(DEBUG_INFO,"mytp parse dts funtion!\n");
	/* reset , irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np,"reset-gpio",0,(enum of_gpio_flags)&pdata->reset_gpio_flags);
	if(pdata->reset_gpio<0){
		mytp_debug(DEBUG_ERROR,"mytp unable to get reset_gpio!\n");
		rc = -EINVAL;
	}
	pdata->irq_gpio = of_get_named_gpio_flags(np,"touch-gpio",0,(enum of_gpio_flags)&pdata->irq_gpio_flags);
	if(pdata->irq_gpio<0){
		mytp_debug(DEBUG_ERROR,"mytp unable to get irq_gpio!\n");
		rc = -EINVAL;
	}
	pdata->mytp_irq = gpio_to_irq(pdata->irq_gpio);
	if(pdata->mytp_irq<0){
		mytp_debug(DEBUG_ERROR,"mytp irq_gpio to mytp_irq failed!\n");
		rc = -EINVAL;
	}
	mytp_debug(DEBUG_INFO,"mytp rst_gpio = %d!\n",pdata->reset_gpio);
	mytp_debug(DEBUG_INFO,"mytp irq_gpio = %d!\n",pdata->irq_gpio);
	mytp_debug(DEBUG_INFO,"mytp mytp_irq = %d!\n",pdata->mytp_irq);
	return rc;	
}

static void mytp_hw_reset(struct mytp_platform_data *pdata)
{
	mytp_debug(DEBUG_INFO,"mytp hw reset!\n");
	gpio_direction_output(pdata->reset_gpio,0);
	gpio_set_value(pdata->reset_gpio,1);
	msleep(10);
	gpio_set_value(pdata->reset_gpio,0);
	msleep(20);
	gpio_set_value(pdata->reset_gpio,1);
}

static int mytp_i2c_send_data(struct i2c_client *client,uint8_t *buf,uint8_t len)
{
	int rc=0;
	rc=i2c_master_send(client,buf,len);
	return rc;
}

static int mytp_i2c_recv_data(struct i2c_client *client,uint8_t *buf,uint8_t len)
{
	int rc=0;
	rc=i2c_master_recv(client,buf,len);
	return rc;
}

static int mytp_poll(struct mytp_platform_data *pdata)
{
	int status=0,retry=10;
	do{
		status=gpio_get_value(pdata->irq_gpio);
		if(status==0) break;
		mytp_debug(DEBUG_MESSAGES,"mytp_poll_interrupt_status = %d!\n",status);
		msleep(50);
		retry--;
	}while(status == 1 && retry > 0);
	mytp_debug(DEBUG_INFO,"mytp_poll_interrupt_status = %d!\n",status);
}

static int mytp_get_data(struct i2c_client *client,uint8_t *cmd,uint8_t *buf,size_t w_size,size_t r_size)
{
	int rc;
	if(buf==NULL)
		return -EINVAL;
	if((i2c_master_send(client,cmd,w_size)) != w_size){
		mytp_debug(DEBUG_ERROR,"%s: i2c_master_send failed!\n",__func__);
		return -EINVAL;
	}
	
}

static int mytp_check_fwmode(struct i2c_client *client,struct mytp_platform_data *pdata)
{
	int len=0;
	int j;
	uint8_t check_fwmode_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x18};
	uint8_t buff[67]={0};
}

static int mytp_read_fwinfo(struct i2c_client *client,struct mytp_platform_data *pdata)
{
	int ret=0;
}

static int mytp_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret=0;
	struct mytp_platform_data *pdata;
	mytp_debug(DEBUG_INFO,"mytp probe start!\n");
	if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C)){
		mytp_debug(DEBUG_ERROR,"i2c check function failed!\n");
		ret = -ENODEV;
		goto err_check_function_failed;
	}

	pdata = kzalloc(sizeof(struct mytp_platform_data),GFP_KERNEL);
	if(pdata == NULL){
		mytp_debug(DEBUG_ERROR,"allocate mytp_platform_data failed!\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	pdata->client = client;

	ret = mytp_parse_dts(&client->dev,pdata);
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"mytp_parse_dts failed!\n");
		ret = -EINVAL;
		goto err_parse_dts_failed;
	}

	mytp_hw_reset(pdata);
	msleep(300);

	mytp_debug(DEBUG_INFO,"mytp probe end!\n");
err_parse_dts_failed:
err_check_function_failed:
err_alloc_data_failed:
	return ret;
}

static int mytp_remove(struct i2c_client *client)
{
	mytp_debug(DEBUG_INFO,"mytp remove start!\n");
	mytp_debug(DEBUG_INFO,"mytp remove end!\n");
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
	mytp_debug(DEBUG_INFO,"mytp init start!\n");
	ret = i2c_add_driver(&mytp_driver);
	if(ret != 0){
		mytp_debug(DEBUG_ERROR,"mytp driver init failed!\n");
	}
	mytp_debug(DEBUG_INFO,"mytp init end!\n");
	return ret;
}

static __exit void mytp_exit(void)
{
	mytp_debug(DEBUG_INFO,"mytp exit start!\n");
	i2c_del_driver(&mytp_driver);
	mytp_debug(DEBUG_INFO,"mytp exit end!\n");	
}

module_init(mytp_init);
module_exit(mytp_exit);

MODULE_AUTHOR("samir <samir@163.com.cn>");
MODULE_DESCRIPTION("TP Driver Demo");
MODULE_LICENSE("GPL");