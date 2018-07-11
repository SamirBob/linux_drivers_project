#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>

#include "my_tp_drivers.h"

static int mytp_parse_dts(struct device *dev,struct mytp_platform_data *pdata)
{
	int rc =0;
	struct device_node *np = dev->of_node;
	mytp_debug(DEBUG_INFO,"mytp parse dts funtion!\n");
	/* reset , irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np,"reset-gpio",0,(enum of_gpio_flags *)&pdata->reset_gpio_flags);
	if(pdata->reset_gpio<0){
		mytp_debug(DEBUG_ERROR,"%s :mytp unable to get reset_gpio!\n",__func__);
		return -EINVAL;
	}
	pdata->irq_gpio = of_get_named_gpio_flags(np,"touch-gpio",0,(enum of_gpio_flags *)&pdata->irq_gpio_flags);
	if(pdata->irq_gpio<0){
		mytp_debug(DEBUG_ERROR,"%s :mytp unable to get irq_gpio!\n",__func__);
		return -EINVAL;
	}
	rc=gpio_request(pdata->reset_gpio,"mytp_reset_gpio");
	if(rc<0){
		mytp_debug(DEBUG_ERROR,"%s :mytp_reset_gpio request gpio failed!\n",__func__);
		return -EINVAL;
	}
	rc=gpio_request(pdata->irq_gpio,"mytp_irq_gpio");
	if(rc<0){
		mytp_debug(DEBUG_ERROR,"%s :mytp_irq_gpio request gpio failed!\n",__func__);
		return -EINVAL;
	}
	pdata->mytp_irq = gpio_to_irq(pdata->irq_gpio);
	if(pdata->mytp_irq<0){
		mytp_debug(DEBUG_ERROR,"%s :mytp irq_gpio to mytp_irq failed!\n",__func__);
		return -EINVAL;
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

static int mytp_i2c_send_data(struct mytp_platform_data * pdata,uint8_t *buf,uint8_t len)
{
	int rc=0;
	rc=i2c_master_send(pdata->client,buf,len);
	return rc;
}

static int mytp_i2c_recv_data(struct mytp_platform_data * pdata,uint8_t *buf,uint8_t len)
{
	int rc=0;
	rc=i2c_master_recv(pdata->client,buf,len);
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
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int mytp_get_data(struct mytp_platform_data * pdata,uint8_t *cmd,uint8_t *buf,size_t w_size,size_t r_size)
{
	int rc;
	if(buf==NULL)
		return -EINVAL;
	if((mytp_i2c_send_data(pdata,cmd,w_size)) != w_size){
		mytp_debug(DEBUG_ERROR,"%s: mytp_i2c_send_data failed!\n",__func__);
		return -EINVAL;
	}
	rc=mytp_poll(pdata);
	if(rc<0){
		mytp_debug(DEBUG_ERROR,"%s: mytp_poll status high!\n",__func__);
	}
	if(r_size <= 0){
		r_size = w_size;
	}
	if((mytp_i2c_recv_data(pdata,buf,r_size)) != r_size){
		mytp_debug(DEBUG_ERROR,"%s: mytp_i2c_send_data failed!\n",__func__);
		return -EINVAL;
	}
	return 0;		
}

static int mytp_check_fwmode(struct mytp_platform_data *pdata)
{
	int ret=0;
	uint8_t check_fwmode_cmd[37]={0x04,0x00,0x23,0x00,0x03,0x18};
	uint8_t buff_recv[67]={0};
	mytp_debug(DEBUG_INFO,"mytp_check_fwmode start!\n");
	ret = mytp_get_data(pdata,check_fwmode_cmd,buff_recv,sizeof(check_fwmode_cmd),sizeof(buff_recv));
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"%s mytp_get_data faile!\n",__func__);
		return -EINVAL;
	}
	if(buff_recv[4]==0x20){
		mytp_debug(DEBUG_INFO,"%s mytp_fw_mode = normal!\n",__func__);
		ret=buff_recv[4];
	}
	return ret;
}

static int mytp_read_fwinfo(struct mytp_platform_data *pdata)
{
	int ret=0;
	int major,minor;
	/* Get FW_VER,FWID_VER,BCODE_VER CMD */
	uint8_t cmd_fw_ver[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x53,0x00,0x00,0x01};
	uint8_t cmd_id_ver[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x53,0xf0,0x00,0x01};
	uint8_t cmd_bc_ver[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x04,0x53,0x10,0x00,0x01};
	/* Get FW TX and RX Number cmd */
	uint8_t cmd_txrx_number[37]={0x04,0x00,0x23,0x00,0x03,0x00,0x06,0x5b,0x00,0x00,0x00};
	uint8_t buff_recv[67]={0};
	mytp_debug(DEBUG_INFO,"mytp_read_fwinfo start!\n");
	ret = mytp_get_data(pdata,cmd_fw_ver,buff_recv,sizeof(cmd_fw_ver),sizeof(buff_recv));
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"mytp read fw ver failed!\n");
		return -EINVAL;
	}else{
		major=((buff_recv[5] & 0x0f) << 4)|((buff_recv[6] & 0xf0) >> 4);
		minor=((buff_recv[6] & 0x0f) << 4)|((buff_recv[7] & 0xf0) >> 4);
		pdata->fwinfo.mytp_fw_ver = major<<8 |minor;
		mytp_debug(DEBUG_INFO,"%s :fw_ver= 0x%4.4x\n",__func__,pdata->fwinfo.mytp_fw_ver);
	}

	ret = mytp_get_data(pdata,cmd_id_ver,buff_recv,sizeof(cmd_id_ver),sizeof(buff_recv));
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"mytp read fw id ver failed!\n");
		return -EINVAL;
	}else{
		major=((buff_recv[5] & 0x0f) << 4)|((buff_recv[6] & 0xf0) >> 4);
		minor=((buff_recv[6] & 0x0f) << 4)|((buff_recv[7] & 0xf0) >> 4);
		pdata->fwinfo.mytp_fw_id= major<<8 |minor;
		mytp_debug(DEBUG_INFO,"%s :fw_id= 0x%4.4x\n",__func__,pdata->fwinfo.mytp_fw_id);
	}

	ret = mytp_get_data(pdata,cmd_bc_ver,buff_recv,sizeof(cmd_bc_ver),sizeof(buff_recv));
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"mytp read fw Bcode ver failed!\n");
		return -EINVAL;
	}else{
		major=((buff_recv[5] & 0x0f) << 4)|((buff_recv[6] & 0xf0) >> 4);
		minor=((buff_recv[6] & 0x0f) << 4)|((buff_recv[7] & 0xf0) >> 4);
		pdata->fwinfo.mytp_bcode_ver= major<<8 |minor;
		mytp_debug(DEBUG_INFO,"%s :fw_bcode= 0x%4.4x\n",__func__,pdata->fwinfo.mytp_bcode_ver);
	}

	ret = mytp_get_data(pdata,cmd_txrx_number,buff_recv,sizeof(cmd_txrx_number),sizeof(buff_recv));
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"mytp read fw tx and rx number failed!\n");
		return -EINVAL;
	}else{
		pdata->fwinfo.mytp_rx_num=buff_recv[6];
		pdata->fwinfo.mytp_tx_num=buff_recv[7];
		mytp_debug(DEBUG_INFO,"%s :fw_rx_number= %d ,fw_tx_number= %d\n",__func__,pdata->fwinfo.mytp_rx_num,pdata->fwinfo.mytp_tx_num);
	}

	pdata->fwinfo.finger_x_resolution_max = (pdata->fwinfo.mytp_tx_num-1)*FINGER_OSR;
	pdata->fwinfo.finger_y_resolution_max = (pdata->fwinfo.mytp_rx_num-1)*FINGER_OSR;
	pdata->fwinfo.activepen_x_resolution_max = (pdata->fwinfo.mytp_tx_num-1)*ACTIVEPEN_OSR;
	pdata->fwinfo.activepen_y_resolution_max = (pdata->fwinfo.mytp_rx_num-1)*ACTIVEPEN_OSR;
	
	mytp_debug(DEBUG_INFO,"%s :finger_x_resolution_max= %d , finger_y_resolution_max= %d\n",__func__,pdata->fwinfo.finger_x_resolution_max,pdata->fwinfo.finger_y_resolution_max);
	mytp_debug(DEBUG_INFO,"%s :activepen_x_resolution_max= %d , activepen_y_resolution_max= %d\n",__func__,pdata->fwinfo.activepen_x_resolution_max,pdata->fwinfo.activepen_y_resolution_max);
	
	return ret;	
}

static int mytp_register_finger_input(struct mytp_platform_data *pdata)
{
	int ret=0;
	mytp_debug(DEBUG_INFO,"mytp register finger input start!\n");
	pdata->finger_input_dev=input_allocate_device();
	if(pdata->finger_input_dev == NULL){
		mytp_debug(DEBUG_ERROR,"mytp input allocate device failed\n");
		return -ENOMEM;
	}
	pdata->finger_input_dev->name = "mytp_finger_touchscreen";
	__set_bit(EV_ABS,pdata->finger_input_dev->evbit);
	__set_bit(EV_SYN,pdata->finger_input_dev->evbit);
	__set_bit(EV_KEY,pdata->finger_input_dev->evbit);
	__set_bit(MT_TOOL_FINGER,pdata->finger_input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,pdata->finger_input_dev->propbit);
#ifdef MYTP_PROTOCOL_B
	input_mt_init_slots(pdata->finger_input_dev,MYTP_FINGER_NUM_MAX,0);
#endif
	input_set_abs_params(pdata->finger_input_dev,ABS_MT_POSITION_X,0,LCM_X_RESOLUTION_MAX,0,0);
	input_set_abs_params(pdata->finger_input_dev,ABS_MT_POSITION_Y,0,LCM_Y_RESOLUTION_MAX,0,0);
	ret=input_register_device(pdata->finger_input_dev);
	if(ret){
		mytp_debug(DEBUG_ERROR,"mytp input register device failed\n");
		return -ENOMEM;
	}
	return ret;
}

static int mytp_register_activepen_input(struct mytp_platform_data *pdata)
{
	int ret=0;
	mytp_debug(DEBUG_INFO,"mytp register activepen input start!\n");
	pdata->activepen_input_dev=input_allocate_device();
	if(pdata->activepen_input_dev== NULL){
		mytp_debug(DEBUG_ERROR,"mytp activepen input allocate device failed\n");
		return -ENOMEM;
	}
	pdata->activepen_input_dev->name = "mytp_activepen_touchscreen";
	__set_bit(EV_ABS,pdata->activepen_input_dev->evbit);
	__set_bit(EV_SYN,pdata->activepen_input_dev->evbit);
	__set_bit(EV_KEY,pdata->activepen_input_dev->evbit);
	__set_bit(MT_TOOL_PEN,pdata->activepen_input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,pdata->activepen_input_dev->propbit);
#ifdef MYTP_PROTOCOL_B
	input_mt_init_slots(pdata->finger_input_dev,MYTP_FINGER_NUM_MAX,0);
#endif
	input_set_abs_params(pdata->activepen_input_dev,ABS_MT_PRESSURE,0,ACTIVEPEN_PRESSURE_MAX,0,0);
	input_set_abs_params(pdata->activepen_input_dev,ABS_MT_POSITION_X,0,LCM_X_RESOLUTION_MAX,0,0);
	input_set_abs_params(pdata->activepen_input_dev,ABS_MT_POSITION_Y,0,LCM_Y_RESOLUTION_MAX,0,0);
	ret=input_register_device(pdata->activepen_input_dev);
	if(ret){
		mytp_debug(DEBUG_ERROR,"mytp activepen input register device failed\n");
		return -ENOMEM;
	}
	return ret;
}

static inline int mytp_finger_parse_xy(uint8_t *data,uint16_t *x,uint16_t *y)
{
	*x=*y=0;
	
	*x = (data[6]);
	*x <<= 8;
	*x |= data[5];

	*y = (data[10]);
	*y <<= 8;
	*y |= data[9];	

	return 0;
}

static inline int mytp_activepen_parse_xy(uint8_t *data,uint16_t *x,uint16_t *y,uint16_t *p)
{
	*x=*y=*p=0;
	
	*x = data[5];
	*x <<= 8;
	*x |= data[4];

	*y = data[7];
	*y <<= 8;
	*y |= data[6];

	*p = data[9];
	*p <<=8;
	*p |= data[8];

	return 0;
}

static void mytp_report_data(struct mytp_platform_data *pdata,uint8_t *buff)
{
	uint16_t x=0,y=0,p=0;
	uint8_t finger_num,finger_down,idx,finger;
	uint8_t activepen_down,activepen_hover,activepen_key;
	switch (buff[0]){
		case MYTP_FINGERS_PKT:
			mytp_debug(DEBUG_INFO,"mytp finger report \n");
			finger_num=buff[1];
			idx=3;
			for(finger=0;finger<finger_num;finger++){
				finger_down = buff[idx]&0x03;				
				if(finger_down == 0x03){
					mytp_finger_parse_xy(&buff[idx],&x,&y);
					x=(x*LCM_X_RESOLUTION_MAX)/(pdata->fwinfo.finger_x_resolution_max);
					y=(y*LCM_Y_RESOLUTION_MAX)/(pdata->fwinfo.finger_y_resolution_max);
					input_report_abs(pdata->finger_input_dev,ABS_MT_POSITION_X,x);
					input_report_abs(pdata->finger_input_dev,ABS_MT_POSITION_Y,y);
					input_report_key(pdata->finger_input_dev,MT_TOOL_FINGER, 1);
					input_mt_sync(pdata->finger_input_dev);
				}else{
					input_report_key(pdata->finger_input_dev,MT_TOOL_FINGER, 0);
					input_mt_sync(pdata->finger_input_dev);
				}
				idx += 11;
			}
			input_sync(pdata->finger_input_dev);
			break;
		case MYTP_ACTIVEPEN_PKT:
			mytp_debug(DEBUG_INFO,"mytp activepen report \n");
			activepen_hover=buff[3]&0x1;
			activepen_key=(buff[3]>>2)&0x7;
			if(activepen_hover){
				mytp_activepen_parse_xy(&buff[0],&x,&y,&p);
				x=(x*(LCM_X_RESOLUTION_MAX)/(pdata->fwinfo.activepen_x_resolution_max);
				y=(y*(LCM_Y_RESOLUTION_MAX)/(pdata->fwinfo.activepen_y_resolution_max);
				input_report_abs(pdata->activepen_input_dev,ABS_MT_PRESSURE,p);
				input_report_abs(pdata->activepen_input_dev,ABS_MT_POSITION_X,x);
				input_report_abs(pdata->activepen_input_dev,ABS_MT_POSITION_Y,y);
				input_report_key(pdata->activepen_input_dev,MT_TOOL_PEN, 1);
				input_mt_sync(pdata->activepen_input_dev);
			}else{
				input_report_key(pdata->activepen_input_dev,MT_TOOL_PEN,0);
				input_mt_sync(pdata->activepen_input_dev);
			}
			input_sync(pdata->activepen_input_dev);
			break;
	}
}

static irqreturn_t mytp_irq_handler(int irq,void *dev_id)
{
	int ret=0;
	int i=0;
	uint8_t buff_recv_1[PACKET_SIZE*2]={0};
	uint8_t buff_recv_2[PACKET_SIZE]={0};
	mytp_debug(DEBUG_INFO,"mytp irq handler start!\n");
	ret=mytp_i2c_recv_data(dev_id,buff_recv_1,sizeof(buff_recv_1));
	if(ret != sizeof(buff_recv_1)){
		mytp_debug(DEBUG_ERROR,"mytp received the packet failed\n");
		return IRQ_HANDLED;
	}
	buff_recv_1[1]=buff_recv_1[62];
	if((buff_recv_1[0]==0x3f) && (buff_recv_1[62]>5)){		
		ret=mytp_i2c_recv_data(dev_id,buff_recv_2,sizeof(buff_recv_2));
		if(ret != sizeof(buff_recv_2)){
			mytp_debug(DEBUG_ERROR,"mytp received the packet failed\n");
			return IRQ_HANDLED;
		}
		for(i=3;i<67;i++){
			buff_recv_1[55+3]=buff_recv_2[i];
		}
	}
	mytp_report_data(dev_id,buff_recv_1);
	return IRQ_HANDLED;
}

static int mytp_register_interrupt(struct mytp_platform_data *pdata)
{
	int ret=0;
	mytp_debug(DEBUG_INFO,"mytp register interrupt start!\n");
	ret=request_threaded_irq(pdata->mytp_irq,NULL,mytp_irq_handler,IRQF_TRIGGER_FALLING|IRQF_ONESHOT,MYTP_DRIVER_NAME,pdata);
	if(ret){
		mytp_debug(DEBUG_ERROR,"mytp register interrupt failed\n");
	}
	return ret;
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
	ret = mytp_check_fwmode(pdata);
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"%s check fwmode faile!\n",__func__);
	}

	ret = mytp_read_fwinfo(pdata);
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"%s read fwinfo faile!\n",__func__);
	}

	ret = mytp_register_finger_input(pdata);
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"%s register finger input faile!\n",__func__);
	}

	ret = mytp_register_activepen_input(pdata);
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"%s register activepen input faile!\n",__func__);
	}

	ret = mytp_register_interrupt(pdata);
	if(ret<0){
		mytp_debug(DEBUG_ERROR,"%s register interrupt faile!\n",__func__);
	}

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