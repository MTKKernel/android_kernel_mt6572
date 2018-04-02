#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <mach/mt_gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>      // might need to get fuel gauge info
#include <linux/power_supply.h>     // might need to get fuel gauge info
#include <mach/lge_miniabb.h>
#include <linux/xlog.h>
#include <linux/kobject.h>

#include <cust_gpio_usage.h>
#include <cust_gpio_boot.h>

#define TESTCODE 1

void charger_enable();

/*                                                                                                */
enum
{
    MINI_ABB_UNINITIALIZE_CABLE     = -1,
    MINI_ABB_NORMAL_MODE            = 0,
    MINI_ABB_FACTORY_MODE           = 1
} MINI_ABB_PTM_MODE;
static int miniabb_is_PTM = MINI_ABB_NORMAL_MODE;

//extern int Is_Not_FactoryCable_PowerOn();
/*                                                                                                */

static int miniabb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int miniabb_i2c_remove(struct i2c_client *client);

static struct i2c_client *miniabb_i2c_client = NULL;
static struct platform_driver miniabb_driver;
static struct miniabb_i2c_data *i2c_data = NULL;
static const struct i2c_device_id miniabb_i2c_id[] = {{MINIABB_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_MINIABB = {I2C_BOARD_INFO(MINIABB_DEV_NAME, 0x44)};
int miniabb_vendor_id=0;
//extern int CAMERA_HW_i2C_init(void);

struct miniabb_i2c_data {
    struct i2c_client *client;
    int blue;
    int green;
    int red;
    int keyleds;
#if TESTCODE
    int main_cam_dvdd;
    int cam_avdd;
    int cam_iovdd;
    int mtk_sensor;
    int chg_current;
    int chg_stop;
#endif
};

static void MINIABB_CheckDevID(struct i2c_client *client)
{
    int tempvalue;

    if((!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))) {
        printk("[MINIABB] I2C check ERROR!!\n");
    }

    tempvalue = 0;
    tempvalue = i2c_smbus_read_word_data(client,0x00);
    if(tempvalue < 0){
        printk("[MINIABB] Check ID error!!\n");
    }
    else {
        printk("[MINIABB]Device ID = [%02x]\n",tempvalue);
    }
}

static int Check_VendorID(struct i2c_client *client)
{
	int tempvalue;

	if((!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))) {
		printk("[MINIABB] I2C check ERROR!!\n");
	}

	tempvalue = 0;
	tempvalue = i2c_smbus_read_byte_data(client,0x00);
	if(tempvalue < 0){
		printk("[MINIABB] Check ID error!!\n");
	}
	else {
		tempvalue = tempvalue >> 4;
		printk("[MINIABB] Vendor ID = [%d]\n", tempvalue);
    }
	
	return tempvalue;
}

static void MINIABB_SetGpio(void)
{
    // i2c0 pin setting
    mt_set_gpio_mode(GPIO_COMMON_I2C_SCL, GPIO_COMMON_I2C_SCL_M_SCL);
    mt_set_gpio_mode(GPIO_COMMON_I2C_SDA, GPIO_COMMON_I2C_SDA_M_SDA);

    // miniabb_int pin setting
    mt_set_gpio_mode(GPIO_MINIABB_INT, GPIO_MINIABB_INT_M_EINT);

    printk("GPIO_SCL[137] Mode:%d\nGPIO_SDA[138] Mode:%d\n"
            ,mt_get_gpio_mode(137),mt_get_gpio_mode(138));

}
int check_miniabb_OVP(void){

	struct i2c_client *client = miniabb_i2c_client;
    int data=0;
	
	if(miniabb_vendor_id!=MINIABB_VENDOR_RICHTEK)
		return 0;


    data = i2c_smbus_read_byte_data(client,INTSTATUS2);
	
    if(data < 0){
        printk("[MINIABB] eoc read error!!\n");
    }
    else {
		if((data>>4)&0x01)
		{
			printk("[MINIABB] detected over voltage \n");
			return 1;
		}
		else
		{
			return 0;
		}
    }

}
EXPORT_SYMBOL(check_miniabb_OVP);


/////////////////////////////////////////////////////////////
/*  CAM LDO control API                                    */
/////////////////////////////////////////////////////////////
#if 1 //MINI_ABB_LDO_FIX
u8 g_reg_en_ctrl = 0;

void miniabb_en_ctrl(int ldo, int on_off)
{
	struct i2c_client *client = miniabb_i2c_client;
	U8 en_ctrl = 0;
	int ret = 0;

	if(client == NULL){
		printk("[MINIABB] i2c client invaild\n");
		return;
    }

	printk("[MINIABB] miniabb_en_ctrl: ldo: 0x%x, on_off: %d\n", ldo, on_off);

	if(on_off) {
		if((g_reg_en_ctrl&ldo) != ldo) {
			en_ctrl = g_reg_en_ctrl;
			en_ctrl |= ldo;
			client->adapter->retries = 5;
			ret = i2c_smbus_write_byte_data(client, EN_CTRL, en_ctrl);
			if(ret < 0) {
				printk("[MINIABB] EN_CTRL write failed: %d\n", ret);
			}
			else {
				g_reg_en_ctrl = en_ctrl;
			}
		}
		else {
			printk("[MINIABB] EN_CTRL alreay enabled: %d, %d\n", ldo, g_reg_en_ctrl);
		}
	}
	else {
		if((g_reg_en_ctrl&ldo) != 0) {
			en_ctrl = g_reg_en_ctrl;
			en_ctrl &= ~ldo;
			client->adapter->retries = 5;
			ret = i2c_smbus_write_byte_data(client, EN_CTRL, en_ctrl);
			if(ret < 0) {
				printk("[MINIABB] EN_CTRL write failed: %d\n", ret);
			}
			else {
				g_reg_en_ctrl = en_ctrl;
			}
		}
		else {
			printk("[MINIABB] EN_CTRL alreay disabled: %d, %d\n", ldo, g_reg_en_ctrl);
		}
	}
}
#endif

//                                                                     
void hi351_cam_power(int on_off)
{
#if 1 //MINI_ABB_LDO_FIX

    u8 LDO_EN = 0;

    LDO_EN = LDO1_EN | LDO2_EN | LDO3_EN;

    miniabb_en_ctrl(LDO_EN, on_off);

#else
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;
    u8 LDO_EN = 0;

    if(client==NULL){
        printk("cam_power client NULL\n");
        return ;
    }

    LDO_EN = LDO1_EN | LDO2_EN | LDO3_EN;

    if(on_off) {
        // setting
        //1V2_CAM_DVDD setting
        data = i2c_smbus_read_word_data(client,LDO_VSET1);
        data |= LDO1_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET1, data);
        //2V8_CAM_AVDD setting
        data = i2c_smbus_read_word_data(client,LDO_VSET1);
        data |= LDO2_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET1, data);
        //1V8_CAM_IOVDD setting
        data = i2c_smbus_read_word_data(client,LDO_VSET2);
        data |= LDO3_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET2, data);

        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

    } else {
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);
    }
#endif
}
EXPORT_SYMBOL(hi351_cam_power);

void hi707_cam_power(int on_off)
{
#if 1 //MINI_ABB_LDO_FIX

    u8 LDO_EN = 0;

    LDO_EN = LDO2_EN | LDO3_EN;

    miniabb_en_ctrl(LDO_EN, on_off);

#else
	struct i2c_client *client = miniabb_i2c_client;
	u8 data;
	u8 LDO_EN = 0;

    if(client==NULL){
        printk("cam_power client NULL\n");
        return ;
    }

    LDO_EN = LDO2_EN | LDO3_EN;

    if(on_off) {
        // setting
        //2V8_CAM_AVDD setting
        data = i2c_smbus_read_word_data(client,LDO_VSET1);
        data |= LDO2_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET1, data);
        //1V8_CAM_IOVDD setting
        data = i2c_smbus_read_word_data(client,LDO_VSET2);
        data |= LDO3_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET2, data);

        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

    } else {
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);
	}
#endif
}
EXPORT_SYMBOL(hi707_cam_power);
//                                                                     

void main_cam_dvdd_power(int on_off)
{
#if 1 //MINI_ABB_LDO_FIX
	miniabb_en_ctrl(LDO1_EN, on_off);
#else
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(client==NULL){
    printk("main_cam_dvdd_power client NULL\n");
        return ;
    }

    if(on_off) {
        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO1_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);
        // setting
        data = i2c_smbus_read_word_data(client,LDO_VSET1);
        data |= LDO1_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET1, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO1_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
#endif
}
EXPORT_SYMBOL(main_cam_dvdd_power);

void cam_avdd_power(int on_off)
{
#if 1 //MINI_ABB_LDO_FIX
	miniabb_en_ctrl(LDO2_EN, on_off);
#else
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(client==NULL){
    printk("cam_avdd_power client NULL\n");
        return ;
    }

    if(on_off) {
        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO2_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);
        // setting
        data = i2c_smbus_read_word_data(client,LDO_VSET1);
        data |= LDO2_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET1, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO2_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
#endif
}
EXPORT_SYMBOL(cam_avdd_power);

void cam_iovdd_power(int on_off)
{
#if 1 //MINI_ABB_LDO_FIX
	miniabb_en_ctrl(LDO3_EN, on_off);
#else
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(client==NULL){
    printk("cam_iovdd_power client NULL\n");
        return ;
    }

    if(on_off) {
        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO3_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);
        // setting
        data = i2c_smbus_read_word_data(client,LDO_VSET2);
        data |= LDO3_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET2, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO3_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
#endif
}
EXPORT_SYMBOL(cam_iovdd_power);

void mtk_sensor_power(int on_off)
{
#if 1 //MINI_ABB_LDO_FIX
	miniabb_en_ctrl(LDO4_EN, on_off);
#else
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(client==NULL){
    printk("mtk_sensor_power client NULL\n");
        return ;
    }

    if(on_off) {
        // enable
        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LDO4_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);
        // setting
        data = i2c_smbus_read_word_data(client,LDO_VSET2);
        data |= LDO4_VSET;
        i2c_smbus_write_byte_data(client,LDO_VSET2, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LDO4_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
#endif
}
EXPORT_SYMBOL(mtk_sensor_power);


/////////////////////////////////////////////////////////////
/*  RGB LED backlight API                                  */
/////////////////////////////////////////////////////////////
void Blue_LED_control(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(on_off) {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LED1_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

        data = i2c_smbus_read_word_data(client,LED_SET);
        data = LED_DIM;
        i2c_smbus_write_byte_data(client,LED_SET, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LED1_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);

    }
}
EXPORT_SYMBOL(Blue_LED_control);

void Green_LED_control(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(on_off) {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LED2_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

        data = i2c_smbus_read_word_data(client,LED_SET);
        data = LED_DIM;
        i2c_smbus_write_byte_data(client,LED_SET, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LED2_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);
    }
}
EXPORT_SYMBOL(Green_LED_control);

void Red_LED_control(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(on_off) {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LED3_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

        data = i2c_smbus_read_word_data(client,LED_SET);
        data = LED_DIM;
        i2c_smbus_write_byte_data(client,LED_SET, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LED3_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);
    }
}
EXPORT_SYMBOL(Red_LED_control);

void Button_LED_control(int on_off)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

    if(on_off) {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data |= LED4_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL, data);

        data = i2c_smbus_read_word_data(client,LED_SET);
        data = LED_DIM;
        i2c_smbus_write_byte_data(client,LED_SET, data);

    } else {

        data = i2c_smbus_read_word_data(client,EN_CTRL);
        data &= ~LED4_EN;
        i2c_smbus_write_byte_data(client,EN_CTRL,data);
    }
}
EXPORT_SYMBOL(Button_LED_control);


/////////////////////////////////////////////////////////////
/*  charging API                                           */
/////////////////////////////////////////////////////////////
void check_miniabb_status(){

	struct i2c_client *client = miniabb_i2c_client;
	int data;

	data = i2c_smbus_read_word_data(client,0x08);
	printk("[Miniabb] %s : %d \n",__func__, data);
	data = i2c_smbus_read_word_data(client,0x09);
	printk("[Miniabb] %s : %d \n",__func__, data);	

}
int check_EOC_status()
{
    struct i2c_client *client = miniabb_i2c_client;
    int data=0;

#if 1

    if((!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))) {
        printk("[MINIABB] I2C check ERROR!!\n");
    }

    data = i2c_smbus_read_byte_data(client,0x06);
    if(data < 0){
        printk("[MINIABB] eoc read error!!\n");
    }
    else {
        data = (data >> 4) & 3;
        printk("[Miniabb] %s : %d \n",__func__, data);
    }

    if(data == 3) {
        return TRUE;
    }
    else {
        return FALSE;
    }

    
#else
    // EOC check
    data = i2c_smbus_read_word_data(client,0x06);

    printk("[Miniabb] %s : %d \n",__func__, data);

    if(data == 48) {
        return TRUE;
    }
    else {
        return FALSE;
    }
#endif

}

void charger_enable()
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */
#if 1
    if (miniabb_is_PTM == MINI_ABB_NORMAL_MODE)
    {
       	//charging block off
        i2c_smbus_write_byte_data(client,CHG_CTRL1, 0x50);
        data = 0xC0; //|= CHGEN|EXPDETEN;
        i2c_smbus_write_byte_data(client,CHG_CTRL1, data);
    }
#else
    //charger enable

    if(miniabb_is_PTM)
    {
        data = 0xE0; //|= CHGEN|EXPDETEN|PTM;// factory mode setting
    } else {
        data = 0xC0; //|= CHGEN|EXPDETEN;
    }

    i2c_smbus_write_byte_data(client,CHG_CTRL1, data);
#endif
/*                                                                                                */
}

//                                                                               
int is_charging_enable(void)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;
    
    // CHGEN check
    data = i2c_smbus_read_word_data(client,CHG_CTRL1);
    
    printk("[Miniabb] %s : %x \n",__func__, data);
    
    if((data & CHGEN)== CHGEN) {
        return TRUE;
    }
    else {
        return FALSE;
    }
}
//                                                                               
#if 1
void set_charger_start_mode(CHG_TYPE chg_type)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */
    if (miniabb_is_PTM == MINI_ABB_NORMAL_MODE)
    {
        /* kisung.song : current/EOC LEVEL setting, 4.35V setting (CHG_VSET)*/
        switch(chg_type) {
            case CHG_TA :
                data = CHG_CURR_800mA |CHG_VSET | EOC_LVL_20PER;
                break;
            case CHG_600 :
                data = CHG_CURR_600mA | CHG_VSET| EOC_LVL_10PER;
                break;
            case CHG_500 :
                data = CHG_CURR_500mA |CHG_VSET | EOC_LVL_10PER;
                break;
            case CHG_USB :
                data = CHG_CURR_400mA |CHG_VSET | EOC_LVL_33PER;
                break;
            case CHG_100 :
                data = CHG_CURR_100mA |CHG_VSET | EOC_LVL_10PER;
                break;
            case CHG_90 :
                data = CHG_CURR_90mA |CHG_VSET | EOC_LVL_10PER;
                break;
            default :
                data = CHG_CURR_400mA |CHG_VSET | EOC_LVL_33PER;
                break;
        }

        i2c_smbus_write_byte_data(client,CHG_CTRL2, data);

        charger_enable();
    }
    printk("[Miniabb] Charging Current %dmA\n",chg_type);
/*                                                                                                */

}
#else
void set_charger_start_mode(int value)
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */
    if (miniabb_is_PTM == MINI_ABB_NORMAL_MODE)
    {
        /* kisung.song : current/EOC LEVEL setting, 4.35V setting (CHG_VSET)*/
        if(value == CHG_TA) {
            data = 0x63|CHG_VSET; //CHG_CURR_700mA|EOC_LVL_20PER;
        }
        else {
            data = 0x25|CHG_VSET; //CHG_CURR_400mA|EOC_LVL_33PER;
        }
        i2c_smbus_write_byte_data(client,CHG_CTRL2, data);

        charger_enable();
    }
    printk("[Miniabb] Charging Current %dmA\n",value);
/*                                                                                                */

}
#endif
EXPORT_SYMBOL(set_charger_start_mode);

void set_charger_factory_mode()
{
#if 0
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */

    /* kisung.song : 4.35V setting (CHG_VSET)  */
    data = 0x91|CHG_VSET; //CHG_CURR_1000mA|EOC_LVL_10PER;
    i2c_smbus_write_byte_data(client,CHG_CTRL2, data);
    data = i2c_smbus_read_word_data(client,CHG_CTRL2);

    miniabb_is_PTM = 1;

    charger_enable();

/*                                                                                                */

    printk("[Miniabb] FACTORY charging :  0x%02x \n",data);
#endif
}
EXPORT_SYMBOL(set_charger_factory_mode);


void set_charger_stop_mode()
{
    struct i2c_client *client = miniabb_i2c_client;
    u8 data;

/*                                                                                                */
    if (miniabb_is_PTM == MINI_ABB_NORMAL_MODE)
    {
        i2c_smbus_write_byte_data(client,CHG_CTRL1, 0x40);
    }
/*                                                                                                */
    printk("[Miniabb]charging off!\n");

}
EXPORT_SYMBOL(set_charger_stop_mode);

/******************************************************************************/

static ssize_t show_ledblue_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->blue);
}

static ssize_t store_ledblue_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        Blue_LED_control(ON);
    } else {
        Blue_LED_control(OFF);
    }
    i2c_data->blue = onoff;

    return count;
}

static ssize_t show_ledgreen_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->green);
}

static ssize_t store_ledgreen_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        Green_LED_control(ON);
    } else {
        Green_LED_control(OFF);
    }
    i2c_data->green = onoff;

    return count;
}

static ssize_t show_ledred_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->red);
}

static ssize_t store_ledred_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        Red_LED_control(ON);
    } else {
        Red_LED_control(OFF);
    }
    i2c_data->red = onoff;

    return count;
}

static ssize_t show_keyleds_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->keyleds);
}

static ssize_t store_keyleds_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        Button_LED_control(ON);
    } else {
        Button_LED_control(OFF);
    }
    i2c_data->keyleds = onoff;

    return count;
}

#if TESTCODE
static ssize_t show_main_cam_dvdd_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->main_cam_dvdd);
}

static ssize_t store_main_cam_dvdd_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        main_cam_dvdd_power(ON);
        printk("main_cam_dvdd power on!\n");
    } else {
        main_cam_dvdd_power(OFF);
        printk("main_cam_dvdd power off!\n");
    }
    i2c_data->cam_avdd = onoff;

    return count;
}

static ssize_t show_cam_avdd_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->cam_avdd);
}

static ssize_t store_cam_avdd_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        cam_avdd_power(ON);
        printk("cam_avdd power on!\n");
    } else {
        cam_avdd_power(OFF);
        printk("cam_avdd power off!\n");
    }
    i2c_data->cam_avdd = onoff;

    return count;
}


static ssize_t show_cam_iovdd_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->cam_iovdd);
}

static ssize_t store_cam_iovdd_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        cam_iovdd_power(ON);
        printk("cam_iovdd power on!\n");
    } else {
        cam_iovdd_power(OFF);
        printk("cam_iovdd power off!\n");
    }
    i2c_data->cam_iovdd = onoff;

    return count;
}


static ssize_t show_mtk_sensor_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->mtk_sensor);
}

static ssize_t store_mtk_sensor_value(struct device_driver *ddri, char *buf, size_t count)
{
    int onoff;

    sscanf(buf, "%d", &onoff);

    if(onoff) {
        mtk_sensor_power(ON);
        printk("mtk_sensor power on!\n");
    } else {
        mtk_sensor_power(OFF);
        printk("mtk_sensor power off!\n");
    }
    i2c_data->mtk_sensor = onoff;

    return count;
}

static ssize_t show_chg_start_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->chg_current);
}

static ssize_t store_chg_start_value(struct device_driver *ddri, char *buf, size_t count)
{
    int chg_curr;

    sscanf(buf, "%d", &chg_curr);

    if(chg_curr == CHG_USB ) {
        set_charger_start_mode(CHG_USB);
    } else {
        set_charger_start_mode(CHG_TA);
    }
    i2c_data->chg_current = chg_curr;

    return count;
}

static ssize_t show_chg_stop_value(struct device_driver *ddri, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_data->chg_stop);
}


static ssize_t store_chg_stop_value(struct device_driver *ddri, char *buf, size_t count)
{
    int chg_stop;

    sscanf(buf, "%d", &chg_stop);

    if(chg_stop) {
        set_charger_stop_mode();
    }
    i2c_data->chg_stop = chg_stop;

    return count;
}

#endif


/******************************************************************************/
static DRIVER_ATTR(ledblue,    S_IWUSR | S_IRUGO | S_IWGRP, show_ledblue_value, store_ledblue_value);
static DRIVER_ATTR(ledgreen,    S_IWUSR | S_IRUGO | S_IWGRP, show_ledgreen_value, store_ledgreen_value);
static DRIVER_ATTR(ledred,    S_IWUSR | S_IRUGO | S_IWGRP, show_ledred_value, store_ledred_value);
static DRIVER_ATTR(keyleds,    S_IWUSR | S_IRUGO | S_IWGRP, show_keyleds_value, store_keyleds_value);
#if TESTCODE
static DRIVER_ATTR(main_cam_dvdd,    S_IWUSR | S_IRUGO | S_IWGRP, show_main_cam_dvdd_value, store_main_cam_dvdd_value);
static DRIVER_ATTR(cam_avdd,    S_IWUSR | S_IRUGO | S_IWGRP, show_cam_avdd_value, store_cam_avdd_value);
static DRIVER_ATTR(cam_iovdd,    S_IWUSR | S_IRUGO | S_IWGRP, show_cam_iovdd_value, store_cam_iovdd_value);
static DRIVER_ATTR(mtk_sensor,    S_IWUSR | S_IRUGO | S_IWGRP, show_mtk_sensor_value, store_mtk_sensor_value);
static DRIVER_ATTR(chg_start,    S_IWUSR | S_IRUGO | S_IWGRP, show_chg_start_value, store_chg_start_value);
static DRIVER_ATTR(chg_stop,    S_IWUSR | S_IRUGO | S_IWGRP, show_chg_stop_value, store_chg_stop_value);
#endif

static struct driver_attribute *miniabb_attr_list[] = {
    &driver_attr_ledblue,   /* blue led control */
    &driver_attr_ledgreen,   /* blue led control */
    &driver_attr_ledred,   /* blue led control */
    &driver_attr_keyleds,   /* button led control */
#if TESTCODE
    &driver_attr_main_cam_dvdd,   /* +1V2_MAIN_CAM_DVDD */
    &driver_attr_cam_avdd,   /* +2V8_CAM_AVDD */
    &driver_attr_cam_iovdd,   /* +1V8_CAM_IOVDD */
    &driver_attr_mtk_sensor,   /* +2V8_MTK_SENSOR */
    &driver_attr_chg_start,   /* start charger */
    &driver_attr_chg_stop,   /* stop charger */
#endif
};

/***********************************************************************************/
static int miniabb_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(miniabb_attr_list)/sizeof(miniabb_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, miniabb_attr_list[idx]))
        {
            printk("driver_create_file (%s) = %d\n", miniabb_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}

/***********************************************************************************/
static int miniabb_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(miniabb_attr_list)/sizeof(miniabb_attr_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }


    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, miniabb_attr_list[idx]);
    }


    return err;
}

/***********************************************************************************/
static int miniabb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {

    printk("[Miniabb] :: miniabb i2c probe \n");

    struct i2c_client *new_client;
    struct miniabb_i2c_data *obj;
    u8 databuf[2];
#if 1 //MINI_ABB_LDO_FIX
	int ret = 0;
#endif

    int err = 0;

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }
    MINIABB_SetGpio();

    memset(obj, 0, sizeof(struct miniabb_i2c_data));

    i2c_data = obj;
    obj->client = client;
    new_client = obj->client;
    i2c_set_clientdata(new_client,obj);

    miniabb_i2c_client = client;

    //printk("miniabb_i2c_client number = 0x%x.\n", miniabb_i2c_client);

    MINIABB_CheckDevID(client);

    if(err = miniabb_create_attr(&miniabb_driver.driver))
    {
        printk("miniabb create attribute err = %d\n", err);
    }
	miniabb_is_PTM = MINI_ABB_NORMAL_MODE;

	#if 0
/*                                                                                                */
    if (Is_Not_FactoryCable_PowerOn() == 0)
    {
        miniabb_is_PTM = MINI_ABB_FACTORY_MODE;
    }
    else
    {
        miniabb_is_PTM = MINI_ABB_NORMAL_MODE;
    }
/*                                                                                                */
	#endif

#if 1 //MINI_ABB_LDO_FIX
	ret = i2c_smbus_read_word_data(client, EN_CTRL);
	if(ret < 0) {
		printk("[MINIABB] EN_CTRL read failed: %d\n", ret);
		g_reg_en_ctrl = 0;
	}
	else {
		g_reg_en_ctrl = (U8)ret;
	}

	ret = i2c_smbus_write_byte_data(client, LDO_VSET1, (LDO1_VSET | LDO2_VSET));
	if(ret < 0) {
		printk("[MINIABB] LDO_VSET1 write failed: %d\n", ret);
	}

	ret = i2c_smbus_write_byte_data(client, LDO_VSET2, (LDO3_VSET | LDO4_VSET));
	if(ret < 0) {
		printk("[MINIABB] LDO_VSET2 write failed: %d\n", ret);
	}
	miniabb_vendor_id=Check_VendorID(client);

	printk("[MINIABB] EN_CTRL: 0x%x\n", g_reg_en_ctrl);
#endif
	
    return 0;

exit:
    miniabb_i2c_client = NULL;
    return err;

}

static int miniabb_i2c_remove(struct i2c_client *client)
{
    int err;

    if(err = miniabb_delete_attr(&miniabb_driver.driver))
    {
        printk("miniabb delete attribute err = %d\n", err);
    }

    return 0;
}


static int miniabb_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    printk("[Miniabb] ::  miniabb_i2c_detect\n");
    strcpy(info->type, MINIABB_DEV_NAME);
    return 0;
}


struct i2c_driver miniabb_i2c_driver = {
    .probe = miniabb_i2c_probe,
    .remove = miniabb_i2c_remove,
    .detect = miniabb_i2c_detect,
    .driver.name = MINIABB_DEV_NAME,
    .id_table = miniabb_i2c_id,
};



static int miniabb_probe(struct platform_device *dev)
{

    printk("[Miniabb] :: charging IC Initialization is done\n");

#if 0
    if(i2c_add_driver(&miniabb_i2c_driver))
    {
        printk("[Miniabb] :: add miniabb i2c driver error !\n");
        return -1;
    }
#endif	

    return 0;
}

static int miniabb_remove(struct platform_device *dev)
{
    //
    i2c_del_driver(&miniabb_i2c_driver);
    return 0;
}
/*
static int miniabb_suspend(struct platform_device *dev)
{
    return 0;
}

static int miniabb_resume(struct platform_device *dev)
{
    return 0;
}
*/


static struct platform_driver miniabb_driver = {
    .probe = miniabb_probe,
    .remove = miniabb_remove,
//    .suspend = miniabb_suspend,
//    .resume = miniabb_resume,
    .driver = {
        .name = "miniabb",
        .owner = THIS_MODULE,
    },
};

static int __init miniabb_driver_init(void) {
    int ret ;

    printk("MiniABB driver init!!\n");

    ret = i2c_register_board_info(1, &i2c_MINIABB, 1);
    if(ret)
    {
        printk("failed to i2c register driver. (%d) \n",ret);
        return ret;
    } else {
        printk("success to i2c register driver. (%d) \n",ret);
    }
    ret = platform_driver_register(&miniabb_driver);
    if(ret)
    {
        printk("failed to register driver. (%d) \n",ret);
        return ret;
    } else {
        printk("success to register driver. (%d) \n",ret);
    }
   if(i2c_add_driver(&miniabb_i2c_driver))
    {
        printk("[Miniabb] :: add miniabb i2c driver error !\n");
        return -1;
    }

    /*In order to prevent sequence that MINI_ABB doesn't be
      initialized and sensor try to start power on sequence for initialization.*/
	  
	#if 0
    CAMERA_HW_i2C_init();
	#endif
    return ret;
}

static void __exit miniabb_driver_exit(void) {
    printk("MiniABB driver exit!!\n");
    platform_driver_unregister(&miniabb_driver);
}

module_init(miniabb_driver_init);
module_exit(miniabb_driver_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("miniabb Driver");
MODULE_LICENSE("GPL");
