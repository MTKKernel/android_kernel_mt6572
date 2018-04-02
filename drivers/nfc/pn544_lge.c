/*
 * Copyright (C) 2010 NXP Semiconductors
 */
 
#include <linux/nfc/pn544_lge.h>
#include <linux/of_gpio.h>

#define MAX_BUFFER_SIZE	512

#define LGE_NFC_READ_IRQ_MODIFY
#define LGE_NFC_GPIO_SBP


#ifdef LGE_NFC_READ_IRQ_MODIFY
bool do_reading = false;//DY_TEST
static bool cancle_read = false;//DY_TEST
#endif

#ifdef LGE_NFC_GPIO_SBP
extern int g_lge_nfc_flag;
#endif

static int	stReadIntFlag;
static struct i2c_client *pn544_client;
static struct pn544_dev *gen_pn544_dev;

#include <linux/dma-mapping.h>

static u8*		I2CDMABuf_va = NULL;
static u32		I2CDMABuf_pa = NULL;
#define LGE_VERSION "pn547_mtk 1.9"

int nfc_i2c_dma_write(struct i2c_client *client, const uint8_t *buf, int len)
{
	int i = 0;
	for(i = 0 ; i < len; i++) {
		I2CDMABuf_va[i] = buf[i];
	}

	if(len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		return i2c_master_send(client, buf, len);
	}
	else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		return i2c_master_send(client, I2CDMABuf_pa, len);
	}
}

int nfc_i2c_dma_read(struct i2c_client *client, uint8_t *buf, int len)
{	
	int i = 0, ret = 0; 	   
	if(len < 8) {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG;
		return i2c_master_recv(client, buf, len);
	}
	else {
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);
		if(ret < 0) {
			return ret;
		}
		for(i = 0; i < len; i++) {
			buf[i] = I2CDMABuf_va[i];
		}
	}
	return ret;
}



static void pn544_parse_dt(struct device *dev, struct pn544_dev *pn544_dev)
{
    struct device_node *np = dev->of_node;

	/* irq gpio info */
    pn544_dev->ven_gpio = GPIO_NFC_VEN;//of_get_named_gpio_flags(np, "nxp,gpio_ven", 0, NULL);
    pn544_dev->firm_gpio = GPIO_NFC_MODE;//of_get_named_gpio_flags(np, "nxp,gpio_mode", 0, NULL);
    pn544_dev->irq_gpio = GPIO_NFC_IRQ;//of_get_named_gpio_flags(np, "nxp,gpio_irq", 0, NULL);
}

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
        //disable_irq_nosync(pn544_get_irq_pin(pn544_dev));
        mt65xx_eint_mask(CUST_EINT_NFC_NUM);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

//static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
static void pn544_dev_irq_handler(void)
{
	struct pn544_dev *pn544_dev = gen_pn544_dev;

	dprintk(PN544_DRV_NAME "%s in\n", __func__);

//	pn544_disable_irq(pn544_dev);
#ifdef LGE_NFC_READ_IRQ_MODIFY
	do_reading=1;//DY_TEST
#endif

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	static char tmp[MAX_BUFFER_SIZE];
	int ret;
	int irq_gpio_val = 0;
    int retry_cnt = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
retry:
	pr_debug("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn544_dev->read_mutex);

	if (!stReadIntFlag) {
//        irq_gpio_val = gpio_get_value(pn544_dev->irq_gpio);
        irq_gpio_val = mt_get_gpio_in(GPIO_NFC_IRQ);
		dprintk(PN544_DRV_NAME ":IRQ GPIO = %d\n", irq_gpio_val);
		if (irq_gpio_val == 0) {
			if (filp->f_flags & O_NONBLOCK) {
				pr_err(PN544_DRV_NAME ":f_falg has O_NONBLOCK. EAGAIN!\n");
				ret = -EAGAIN;
				goto fail;
			}

//			pn544_dev->irq_enabled = true;
#ifdef LGE_NFC_READ_IRQ_MODIFY
		do_reading=0;//DY_TEST
#endif
//            enable_irq(pn544_get_irq_pin(pn544_dev));
            mt65xx_eint_unmask(CUST_EINT_NFC_NUM);
#ifdef LGE_NFC_READ_IRQ_MODIFY
		ret = wait_event_interruptible(pn544_dev->read_wq, do_reading);
#else
			ret = wait_event_interruptible(pn544_dev->read_wq,
					gpio_get_value(pn544_dev->irq_gpio));
#endif
//			pn544_disable_irq(pn544_dev);
			//dprintk(PN544_DRV_NAME ":wait_event_interruptible : %d\n", ret);
#ifdef LGE_NFC_READ_IRQ_MODIFY
        //DY_TEST
        if(cancle_read == true)
        {
            cancle_read = false;
            ret = -1;
            goto fail;
        }
#endif
			if (ret)
				goto fail;
		}
	}

    dprintk("%s i2c_master_recv comes\n", __func__);

	/* Read data */
	memset(tmp, 0x00, MAX_BUFFER_SIZE);
//	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	ret = nfc_i2c_dma_read(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);


	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        if(retry_cnt++<=3) {
            pr_err("%s goto retry %d time\n", __func__, retry_cnt);
            msleep(10);
            goto retry;
        }
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

    dprintk("%s out wiht %d byte(s)\n", __func__, ret);

	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	static char tmp[MAX_BUFFER_SIZE];
	int ret;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(tmp, 0x00, MAX_BUFFER_SIZE);
	if (copy_from_user(tmp, buf, count)) {
		pr_err(PN544_DRV_NAME ":%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	dprintk("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */

//	ret = i2c_master_send(pn544_dev->client, tmp, count);
    ret = nfc_i2c_dma_write(pn544_dev->client, tmp, count);

	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	dprintk(PN544_DRV_NAME ":write: pn544_write return =:%d\n", ret);
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = i2c_get_clientdata(pn544_client);
	pr_info("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}
static int pn544_dev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static long pn544_dev_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/*
			power on with firmware download (requires hw reset)
			*/
			pr_info(PN544_DRV_NAME ":%s power on with firmware\n", __func__);

			//gpio_set_value(pn544_dev->ven_gpio, 1);
            mt_set_gpio_out(GPIO_NFC_VEN, GPIO_OUT_ONE);
			//gpio_set_value(pn544_dev->firm_gpio, 1);
			mt_set_gpio_out(GPIO_NFC_MODE, GPIO_OUT_ONE);
			msleep(10);
			//gpio_set_value(pn544_dev->ven_gpio, 0);
			mt_set_gpio_out(GPIO_NFC_VEN, GPIO_OUT_ZERO);
			msleep(10);
			//gpio_set_value(pn544_dev->ven_gpio, 1);
			mt_set_gpio_out(GPIO_NFC_VEN, GPIO_OUT_ONE);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			pr_info(PN544_DRV_NAME ":%s power on\n", __func__);

			//gpio_set_value(pn544_dev->firm_gpio, 0);
			mt_set_gpio_out(GPIO_NFC_MODE, GPIO_OUT_ZERO);
			//gpio_set_value(pn544_dev->ven_gpio, 1);
			mt_set_gpio_out(GPIO_NFC_VEN, GPIO_OUT_ONE);
			msleep(10);
		} else  if (arg == 0) {
			/* power off */
			pr_info(PN544_DRV_NAME ":%s power off\n", __func__);
			//gpio_set_value(pn544_dev->firm_gpio, 0);
			mt_set_gpio_out(GPIO_NFC_MODE, GPIO_OUT_ZERO);
			//gpio_set_value(pn544_dev->ven_gpio, 0);
			mt_set_gpio_out(GPIO_NFC_VEN, GPIO_OUT_ZERO);
			msleep(10);
#ifdef LGE_NFC_READ_IRQ_MODIFY
		} else if (arg == 3) {//DY_TEST
			pr_info("%s Read Cancle\n", __func__);
            cancle_read = true;
            do_reading = 1;
        	wake_up(&pn544_dev->read_wq);
#endif
		} else {
				pr_err("%s bad arg %ld\n", __func__, arg);
			return -EINVAL;
		}
		break;
	case PN544_INTERRUPT_CMD:
		{
			/*
			pn544_disable_irq = level;
			*/
			dprintk(PN544_DRV_NAME ":ioctl: pn544_interrupt enable level:%ld\n", arg);
			break;
		}
	case PN544_READ_POLLING_CMD:
		{
			stReadIntFlag = arg;
			dprintk(PN544_DRV_NAME ":ioctl: pn544_polling flag set:%ld\n", arg);
			break;
		}
	default:
		pr_err("%s bad ioctl %d\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.release = pn544_dev_release,
	.unlocked_ioctl = pn544_dev_unlocked_ioctl,
};

static int pn544_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
    int ret;
    struct pn544_dev *pn544_dev = NULL;
    pn544_client = client;
    pr_info(PN544_DRV_NAME " " LGE_VERSION"\n");
    pr_info(PN544_DRV_NAME ": pn544_probe() start\n");

    pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
    if (pn544_dev == NULL) {
        dev_err(&client->dev,
                "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }
    gen_pn544_dev=pn544_dev;
    pn544_parse_dt(&client->dev, pn544_dev);

    pn544_dev->client   = client;
    pr_info(PN544_DRV_NAME ":IRQ : %d  VEN : %d  FIRM : %d\n",
            pn544_dev->irq_gpio, pn544_dev->ven_gpio, pn544_dev->firm_gpio);
    
    /* irq_gpio setup */
    pr_info("%s irq_gpio setup\n", __func__);
    mt_set_gpio_mode(GPIO_NFC_IRQ, GPIO_NFC_IRQ_M_EINT);
    mt_set_gpio_dir(GPIO_NFC_IRQ, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_NFC_IRQ, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_NFC_IRQ, GPIO_PULL_DOWN);

    /* ven_gpio setup */
    pr_info("%s ven_gpio setup\n", __func__);
    mt_set_gpio_mode(GPIO_NFC_VEN, GPIO_NFC_VEN_M_GPIO);
    mt_set_gpio_dir(GPIO_NFC_VEN, GPIO_DIR_OUT);

    /* mode_gpio setup */
    pr_info("%s mode_gpio setup\n", __func__);
    mt_set_gpio_mode(GPIO_NFC_MODE, GPIO_NFC_MODE_M_GPIO);
    mt_set_gpio_dir(GPIO_NFC_MODE, GPIO_DIR_OUT);

    pr_info("%s irq_gpio setup\n", __func__);
    mt65xx_eint_set_sens(CUST_EINT_NFC_NUM, CUST_EINT_LEVEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_NFC_NUM, CUST_EINT_NFC_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_NFC_NUM, CUST_EINT_NFC_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH, pn544_dev_irq_handler, 0);
//    mt65xx_eint_unmask(CUST_EINT_NFC_NUM);

	/* init mutex and queues */
    init_waitqueue_head(&pn544_dev->read_wq);
    mutex_init(&pn544_dev->read_mutex);
    spin_lock_init(&pn544_dev->irq_enabled_lock);

    pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
    pn544_dev->pn544_device.name = PN544_DRV_NAME;
    pn544_dev->pn544_device.fops = &pn544_dev_fops;

    ret = misc_register(&pn544_dev->pn544_device);
    if (ret) {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }

    pn544_disable_irq(pn544_dev);
    i2c_set_clientdata(client, pn544_dev);
    pr_info(PN544_DRV_NAME ": pn544_probe() end\n");
    return 0;

err_request_irq_failed:
    misc_deregister(&pn544_dev->pn544_device);

err_misc_register:
    mutex_destroy(&pn544_dev->read_mutex);
    gpio_free(pn544_dev->firm_gpio);

err_firm:
    gpio_free(pn544_dev->ven_gpio);

err_ven:
    gpio_free(pn544_dev->irq_gpio);

err_int:
    kfree(pn544_dev);

err_exit:
    pr_err(PN544_DRV_NAME ": pn544_dev is null\n");
    pr_err(PN544_DRV_NAME ": pn544_probe() end with error!\n");

	return ret;
}

static __devexit int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(pn544_dev->client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
    gpio_free(pn544_dev->firm_gpio);
	gpio_free(pn544_dev->ven_gpio);
    gpio_free(pn544_dev->irq_gpio);
	kfree(pn544_dev);

	return 0;
}

static void pn544_shutdown(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;
	// Get PN544 Device Structure data
	pn544_dev = i2c_get_clientdata(client);

//	pn544_shutdown_cb(pn544_dev);
	return;
}

static const struct i2c_device_id pn544_id[] = {
	{ PN544_DRV_NAME, 0 },
	{ }
};

static struct of_device_id pn547_match_table[] = {
	{ .compatible = "nxp,pn547",},
	{ },
};

static struct i2c_driver pn544_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = PN544_DRV_NAME,
		.of_match_table = pn547_match_table,
	},
	.probe = pn544_probe,
	.remove = __devexit_p(pn544_remove),
	.shutdown	= pn544_shutdown,
	.id_table = pn544_id,
};

static int __init pn544_dev_init(void)
{
    pr_info("%s in\n", __func__);
#ifdef LGE_NFC_GPIO_SBP
    if(g_lge_nfc_flag != 1)
    {
        pr_info("%s is stop due to SBP %d\n", __func__, g_lge_nfc_flag);
        return 0;
    }
#endif
	int ret = 0;

    I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
    if(!I2CDMABuf_va) {
        pr_err(PN544_DRV_NAME ": Allocate NFC DMA I2C Buffer failed!\n");
        return ENOMEM;
    }

	pr_info("Loading pn544 driver\n");
	/* I2C Driver connection */
    i2c_register_board_info(1, &i2c_nxp_nfc_info, 1);

	ret = i2c_add_driver(&pn544_driver);
	if (ret < 0) {
		printk("[NFC]failed to i2c_add_driver\n");
	}
	pr_info("Loading pn544 or pn547 driver Success! \n");

    return ret;
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
    if(I2CDMABuf_va) {
        dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
        I2CDMABuf_va = NULL;
        I2CDMABuf_pa = 0;
    }
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_DEVICE_TABLE(i2c, pn544_id);
MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
