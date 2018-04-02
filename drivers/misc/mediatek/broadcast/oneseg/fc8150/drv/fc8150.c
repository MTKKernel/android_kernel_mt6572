#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#if !defined(CONFIG_ARCH_MT6572)    // MediaTek platform
#include <linux/gpio.h>
#endif //defined(CONFIG_ARCH_MT6572) // MediaTek platform
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/module.h>
//#include <mach/board_lge.h>

#include "fc8150.h"
#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8150_regs.h"
#include "fc8150_isr.h"
#include "fci_hal.h"

#include <linux/wakelock.h>
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
#include <mach/mt_gpio.h>
#include <mach/eint.h>

#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>    // FC8150 CORE  POWER LDO
#endif //defined(CONFIG_ARCH_MT6572) // MediaTek platform

#if defined(CONFIG_MACH_MSM8974_G2_DCM) || defined(CONFIG_MACH_MSM8974_G2_KDDI)
#include <linux/clk.h>
struct clk *clk;
static u8 use_pm8941_xo_a2_192000;
#endif
u32 bbm_xtal_freq;

ISDBT_INIT_INFO_T *hInit;

u32 totalTS=0;
u32 totalErrTS=0;
unsigned char ch_num = 0;

u8 scan_mode;

extern int g_lge_dtv_flag;
ISDBT_MODE driver_mode = ISDBT_POWEROFF;

int isdbt_open (struct inode *inode, struct file *filp);
long isdbt_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);
int isdbt_release (struct inode *inode, struct file *filp);
ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);

struct wake_lock oneseg_wakelock;
static wait_queue_head_t isdbt_isr_wait;

#define RING_BUFFER_SIZE    (128 * 1024)

//GPIO(RESET & INTRRUPT) Setting
#define FC8150_NAME        "broadcast1"

#define GPIO_ISDBT_IRQ 28
#define GPIO_ISDBT_PWR_EN 22
#define GPIO_ISDBT_RST 23

static DEFINE_MUTEX(ringbuffer_lock);

void isdbt_hw_setting(void)
{
#if defined(CONFIG_MACH_MSM8974_G2_KDDI)
{
    use_pm8941_xo_a2_192000 = 0;
    bbm_xtal_freq = 26000;
    PRINTF(hInit, "[1seg] A1-KDDI : %d, xtal_freq : %d\n",use_pm8941_xo_a2_192000, bbm_xtal_freq);
}
#elif defined(CONFIG_MACH_MSM8974_G2_DCM)
    if (lge_get_board_revno() >= HW_REV_D) {
        use_pm8941_xo_a2_192000 = 1;
        bbm_xtal_freq = 19200;
        PRINTF(hInit, "[1seg] A1-DCM rev.D or later version: %d, xtal_freq : %d\n",use_pm8941_xo_a2_192000, bbm_xtal_freq);
    }
    else {
        use_pm8941_xo_a2_192000 = 0;
        bbm_xtal_freq = 26000;
        PRINTF(hInit, "[1seg] A1-DCM rev.C : %d, xtal_freq : %d\n",use_pm8941_xo_a2_192000, bbm_xtal_freq);
    }
#else
{
    bbm_xtal_freq = 26000;
}
#endif
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
    //PWR Enable
    PRINTF(0,"[1seg][MTK] GPIO_ISDBT_PWR_EN Port request!!!\n");
    mt_set_gpio_mode(GPIO_1SEG_EN, GPIO_1SEG_EN_M_GPIO);
    mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_1SEG_EN,GPIO_PULL_DOWN);
    udelay(50);
    mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);

    // Interrupt  - Setting
    PRINTF(0,"[1seg][MTK] ISDBT_IRQ_INT Port request!!!\n");
    mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_GPIO);
    mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);

    // Reset
    PRINTF(0,"[1seg][MTK] GPIO_ISDBT_RST Port request set zero!!!\n");
    udelay(50);
    mt_set_gpio_mode(GPIO_1SEG_RESET_N,  GPIO_1SEG_RESET_N_M_GPIO);
    mt_set_gpio_pull_enable(GPIO_1SEG_RESET_N, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_1SEG_RESET_N,GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_1SEG_RESET_N, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);

    //SPI MISO
    mt_set_gpio_pull_enable(GPIO100, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO100,GPIO_PULL_DOWN);

    //SPI MOSI
    mt_set_gpio_pull_enable(GPIO99, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO99,GPIO_PULL_DOWN);

    //SPI CLK
    mt_set_gpio_pull_enable(GPIO98, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO98,GPIO_PULL_DOWN);

    //SPI CS
    mt_set_gpio_pull_enable(GPIO97, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO97,GPIO_PULL_DOWN);
#else //CONFIG_ARCH_MT6572
    if(gpio_request(GPIO_ISDBT_PWR_EN, "ISDBT_EN"))
        PRINTF(0,"[1seg] GPIO_ISDBT_PWR_EN Port request error!!!\n");
    udelay(50);
    gpio_direction_output(GPIO_ISDBT_PWR_EN, 0);

    if(gpio_request(GPIO_ISDBT_IRQ, "ISDBT_IRQ_INT"))
        PRINTF(0,"[1seg] ISDBT_IRQ_INT Port request error!!!\n");

    gpio_direction_input(GPIO_ISDBT_IRQ);

    if(gpio_request(GPIO_ISDBT_RST, "ISDBT_RST"))
        PRINTF(0,"[1seg] GPIO_ISDBT_RST Port request error!!!\n");
    udelay(50);
    gpio_direction_output(GPIO_ISDBT_RST, 1);
#endif //CONFIG_ARCH_MT6572
}

//POWER_ON & HW_RESET & INTERRUPT_CLEAR
u8 isdbt_hw_init(void)
{
    int i=0;

    while(driver_mode == ISDBT_DATAREAD)
    {
        msWait(100);
        if(i++>5)
            break;
    }

#if defined(CONFIG_MACH_MSM8974_G2_DCM) || defined(CONFIG_MACH_MSM8974_G2_KDDI)
    if (use_pm8941_xo_a2_192000) {
        if ( !IS_ERR_OR_NULL(clk) )
        {
            int ret = -1;
            ret = clk_prepare_enable(clk);
            if (ret) {
                PRINTF(0,"[1seg] LGE_BROADCAST_DMB_IOCTL_ON enable clock error!!!\n");
                return BBM_NOK;
            }
        }
    }
#endif
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
    PRINTF(0, "[1seg][MTK] isdbt_hw_init\n");
    hwPowerOn(MT6323_POWER_LDO_VCAM_IO, VOL_1800, "1V8_DTV");    // IO power On
    msWait(2);
    mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ONE);
    mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ONE);
    msWait(10);
    mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);
    msWait(5);
    mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ONE);    // current leackage
    msWait(2);

    //Interrupt
    mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_DISABLE);

    //SPI MISO
    mt_set_gpio_pull_enable(GPIO100, GPIO_PULL_DISABLE);

    //SPI MOSI
    mt_set_gpio_pull_enable(GPIO99, GPIO_PULL_DISABLE);

    //SPI CLK
    mt_set_gpio_pull_enable(GPIO98, GPIO_PULL_DISABLE);

    //SPI CS
    mt_set_gpio_pull_enable(GPIO97, GPIO_PULL_DISABLE);
    mt_set_gpio_mode(GPIO97, GPIO_MODE_01);    //GPIO_MATV_I2S_WS_PIN_M_SPI_CS   GPIO_MODE_01
    mt_set_gpio_dir(GPIO97, GPIO_DIR_IN);
#else //CONFIG_ARCH_MT6572
    PRINTF(0, "[1seg] isdbt_hw_init \n");
    gpio_set_value(GPIO_ISDBT_RST, 1);
    gpio_set_value(GPIO_ISDBT_PWR_EN, 1);
    msWait(10);
    gpio_set_value(GPIO_ISDBT_RST, 0);
    msWait(5);
    gpio_set_value(GPIO_ISDBT_RST, 1);    // current leackage
    msWait(2);
#endif //CONFIG_ARCH_MT6572
    driver_mode = ISDBT_POWERON;
    wake_lock(&oneseg_wakelock);

    return BBM_OK;
}

//POWER_OFF
void isdbt_hw_deinit(void)
{
    int ret = -1;
    // disable irq
    mt65xx_eint_mask(CUST_EINT_DTV_NUM);
    driver_mode = ISDBT_POWEROFF;
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
    PRINTF(0, "[1seg][MTK] isdbt_hw_deinit\n");

    // pin setting
    // LDO EN
    mt_set_gpio_pull_enable(GPIO_1SEG_EN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_1SEG_EN,GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_1SEG_EN, GPIO_DIR_OUT);
    ret = mt_set_gpio_out(GPIO_1SEG_EN, GPIO_OUT_ZERO);
    PRINTF(0, "[1seg][MTK] mt_set_gpio_out GPIO_1SEG_EN(%d)\n", ret);

    // Reset
    mt_set_gpio_pull_enable(GPIO_1SEG_RESET_N, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_1SEG_RESET_N,GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_1SEG_RESET_N, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);

    // Interrupt
    mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_1SEG_INT,GPIO_PULL_DOWN);

    //SPI MISO
    mt_set_gpio_pull_enable(GPIO100, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO100,GPIO_PULL_DOWN);

    //SPI MOSI
    mt_set_gpio_pull_enable(GPIO99, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO99,GPIO_PULL_DOWN);

    //SPI CLK
    mt_set_gpio_pull_enable(GPIO98, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO98,GPIO_PULL_DOWN);

    //SPI CS
    mt_set_gpio_mode(GPIO97, GPIO_MODE_00); //GPIO_MATV_I2S_WS_PIN_M_GPIO
    mt_set_gpio_pull_enable(GPIO97, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO97,GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO97, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO97, GPIO_OUT_ZERO);

    // IO power Off
    ret = hwPowerDown(MT6323_POWER_LDO_VCAM_IO, "1V8_DTV");
    PRINTF(0, "[1seg][MTK] hwPowerDown MT6323_POWER_LDO_VCAM_IO(%d)\n", ret);
#else //CONFIG_ARCH_MT6572
    gpio_set_value(GPIO_ISDBT_PWR_EN, 0);
#endif //CONFIG_ARCH_MT6572

    wake_unlock(&oneseg_wakelock);
#if defined(CONFIG_MACH_MSM8974_G2_DCM) || defined(CONFIG_MACH_MSM8974_G2_KDDI)
    if (use_pm8941_xo_a2_192000) {
        if ( !IS_ERR_OR_NULL(clk) )
        {
            clk_disable_unprepare(clk);
        }
    }
#endif
}

u8 irq_error_cnt;
static u8 isdbt_isr_sig=0;
static struct task_struct *isdbt_kthread = NULL;
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
void isdbt_irq(void)
#else
static irqreturn_t isdbt_irq(int irq, void *dev_id)
#endif //CONFIG_ARCH_MT6572
{
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
    //mt65xx_eint_mask(CUST_EINT_DTV_NUM);
#endif //CONFIG_ARCH_MT6572

    //PRINTF(0, "[1seg] isdbt_irq!!!\n");

    if(driver_mode == ISDBT_POWEROFF) {
        PRINTF(0, "[1seg] fc8150 isdbt_irq : abnormal Interrupt occurred fc8150 power off state.cnt : %d\n", irq_error_cnt);
        irq_error_cnt++;
    }
    else {
        isdbt_isr_sig++;
        wake_up(&isdbt_isr_wait);
    }
    mt65xx_eint_unmask(CUST_EINT_DTV_NUM);
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
    return;
#else
    return IRQ_HANDLED;
#endif //CONFIG_ARCH_MT6572
}

int data_callback(u32 hDevice, u8 *data, int len)
{
    ISDBT_INIT_INFO_T *hInit;
    struct list_head *temp;
    int i;

    totalTS +=(len/188);

    for(i=0;i<len;i+=188)
    {
        if((data[i+1]&0x80)||data[i]!=0x47)
            totalErrTS++;
    }

    hInit = (ISDBT_INIT_INFO_T *)hDevice;
    list_for_each(temp, &(hInit->hHead))
    {
        ISDBT_OPEN_INFO_T *hOpen;

        hOpen = list_entry(temp, ISDBT_OPEN_INFO_T, hList);
        //PRINTF(0, "[1seg] fc8150 data_callback : type : %d, [0x%x][0x%x][0x%x][0x%x]\n", hOpen->isdbttype, data[0], data[1], data[2], data[3]);

        if(hOpen->isdbttype == TS_TYPE)
        {
            mutex_lock(&ringbuffer_lock);
            if(fci_ringbuffer_free(&hOpen->RingBuffer) < len )
            {
                mutex_unlock(&ringbuffer_lock);
                PRINTF(0, "[1seg] fc8150 data_callback : ring buffer is full\n");
                return 0;
            }

            fci_ringbuffer_write(&hOpen->RingBuffer, data, len);
            wake_up_interruptible(&(hOpen->RingBuffer.queue));

            mutex_unlock(&ringbuffer_lock);
        }
    }

    return 0;
}

static int isdbt_thread(void *hDevice)
{
    ISDBT_INIT_INFO_T *hInit = (ISDBT_INIT_INFO_T *)hDevice;

    set_user_nice(current, -20);

    PRINTF(hInit, "isdbt_kthread enter\n");

    BBM_TS_CALLBACK_REGISTER((u32)hInit, data_callback);

    init_waitqueue_head(&isdbt_isr_wait);

    while(1)
    {
        wait_event_interruptible(isdbt_isr_wait, isdbt_isr_sig || kthread_should_stop());
        if (irq_error_cnt >= 1){
            PRINTF(0, "[1seg] fc8150 isdbt_irq : abnormal Interrupt occurred fc8150 power off state.cnt : %d\n", irq_error_cnt);
            irq_error_cnt = 0;
        }
        if(driver_mode == ISDBT_POWERON)
        {
            driver_mode = ISDBT_DATAREAD;
            BBM_ISR(hInit);
            driver_mode = ISDBT_POWERON;
            //PRINTF(0, "[1seg] isdbt_thread\n");
        }

        if(isdbt_isr_sig>0)
        {
            isdbt_isr_sig--;
        }

        if (kthread_should_stop())
            break;
    }

    BBM_TS_CALLBACK_DEREGISTER();

    PRINTF(hInit, "[1seg] isdbt_kthread exit\n");

    return 0;
}

static struct file_operations isdbt_fops =
{
    .owner        = THIS_MODULE,
    .unlocked_ioctl    = isdbt_ioctl,
    .open        = isdbt_open,
    .read        = isdbt_read,
    .release    = isdbt_release,
};

static struct miscdevice fc8150_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = FC8150_NAME,
    .fops = &isdbt_fops,
};

int isdbt_open (struct inode *inode, struct file *filp)
{
    int ret = 0;
    ISDBT_OPEN_INFO_T *hOpen;
    static u8 temp[RING_BUFFER_SIZE];
    //enable irq
    //mt65xx_eint_unmask(CUST_EINT_DTV_NUM);
    PRINTF(hInit, "[1seg] isdbt file descriptor open2 mt65xx_eint_unmask\n");
/*
    ret = mt_set_gpio_mode(GPIO_1SEG_RESET_N,  GPIO_1SEG_RESET_N_M_GPIO);
    PRINTF(0, "[1seg][MTK] mt_set_gpio_mode test!! ret = %d\n", ret);
    ret = mt_set_gpio_pull_enable(GPIO_1SEG_RESET_N, GPIO_PULL_ENABLE);
    PRINTF(0, "[1seg][MTK] mt_set_gpio_pull_enable test!! ret = %d\n", ret);
    ret = mt_set_gpio_dir(GPIO_1SEG_RESET_N, GPIO_DIR_OUT);
    PRINTF(0, "[1seg][MTK] mt_set_gpio_dir test!! ret = %d\n", ret);
    msWait(10);
    ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ONE);
    PRINTF(0, "[1seg][MTK] GPIO_ISDBT_RST test 1 ret = %d\n", ret);
    msWait(10);
    ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);
    PRINTF(0, "[1seg][MTK] GPIO_ISDBT_RST test 0 ret = %d\n", ret);
    msWait(10);
    ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ONE);
    //PRINTF(0, "[1seg][MTK] GPIO_ISDBT_RST test 1 ret = %d\n", ret);
    //msWait(10);
    //ret = mt_set_gpio_out(GPIO_1SEG_RESET_N, GPIO_OUT_ZERO);
    //PRINTF(0, "[1seg][MTK] GPIO_ISDBT_RST test 0 ret = %d\n", ret);
*/
    hOpen = (ISDBT_OPEN_INFO_T *)kmalloc(sizeof(ISDBT_OPEN_INFO_T), GFP_KERNEL);

    //hOpen->buf = (u8 *)kmalloc(RING_BUFFER_SIZE, GFP_KERNEL);
    hOpen->buf = temp;
    hOpen->isdbttype = 0;

    list_add(&(hOpen->hList), &(hInit->hHead));

    hOpen->hInit = (HANDLE *)hInit;
/*
    if(hOpen->buf == NULL)
    {
        PRINTF(hInit, "[1seg] ring buffer malloc error\n");
        return -ENOMEM;
    }
*/
    fci_ringbuffer_init(&hOpen->RingBuffer, hOpen->buf, RING_BUFFER_SIZE);

    filp->private_data = hOpen;

    return 0;
}

 ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    s32 avail;
    s32 non_blocking = filp->f_flags & O_NONBLOCK;
    ISDBT_OPEN_INFO_T *hOpen = (ISDBT_OPEN_INFO_T*)filp->private_data;
    struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
    ssize_t len, total_len = 0;

    if (!cibuf->data || !count)
    {
        PRINTF(hInit, "[1seg] return 0\n");
        return 0;
    }

    if (non_blocking && (fci_ringbuffer_empty(cibuf)))
    {
        //PRINTF(hInit, "[1seg] return EWOULDBLOCK\n");
        return -EWOULDBLOCK;
    }

    mutex_lock(&ringbuffer_lock);

    avail = fci_ringbuffer_avail(cibuf);

    if (count >= avail)
        len = avail;
    else
        len = count - (count % 188);

    total_len = fci_ringbuffer_read_user(cibuf, buf, len);

    mutex_unlock(&ringbuffer_lock);

    return total_len;
}

static  ssize_t ioctl_isdbt_read(ISDBT_OPEN_INFO_T *hOpen  ,void __user *arg)
{
    struct broadcast_dmb_data_info __user* puserdata = (struct broadcast_dmb_data_info  __user*)arg;
    int ret = -ENODEV;
    size_t count;
    char *buf;

#if 0
    DMB_BB_HEADER_TYPE dmb_header;
    static int read_count = 0;
#endif

    s32 avail;
    struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
    ssize_t len, total_len = 0;

#if 0
    buf = puserdata->data_buf + sizeof(DMB_BB_HEADER_TYPE);
    count = puserdata->data_buf_size - sizeof(DMB_BB_HEADER_TYPE);
    count = (count/188)*188;
#endif

    buf = puserdata->data_buf;
    count = puserdata->data_buf_size;
    count = (count/188)*188;

    if (!cibuf->data || !count)
    {
        PRINTF(hInit, "[1seg] ioctl_isdbt_read return 0\n");
        return 0;
    }

    if ( fci_ringbuffer_empty(cibuf) )
    {
        //PRINTF(hInit, "[1seg] return fci_ringbuffer_empty EWOULDBLOCK\n");
        return -EWOULDBLOCK;
    }

    mutex_lock(&ringbuffer_lock);
    avail = fci_ringbuffer_avail(cibuf);

    if (count >= avail)
        len = avail;
    else
        len = count - (count % 188);

    total_len = fci_ringbuffer_read_user(cibuf, buf, len);
    mutex_unlock(&ringbuffer_lock);

    if (total_len > 0) {
        puserdata->copied_size = total_len;
        puserdata->packet_cnt = total_len / 188;
        ret = BBM_OK;
    }

    return ret;
}

int isdbt_release (struct inode *inode, struct file *filp)
{
    ISDBT_OPEN_INFO_T *hOpen;

    hOpen = filp->private_data;

    hOpen->isdbttype = 0;

    list_del(&(hOpen->hList));

    //kfree(hOpen->buf);
    kfree(hOpen);

    return 0;
}

int fc8150_if_test(void)
{
    int res=0;
    int i;
    u16 wdata=0;
    u32 ldata=0;
    u8 data=0;
    u8 temp = 0;

    PRINTF(0, "[1seg] fc8150_if_test Start!!!\n");
    for(i=0;i<1000;i++) {
        BBM_BYTE_WRITE(0, 0xa4, i&0xff);
        BBM_BYTE_READ(0, 0xa4, &data);
        if((i&0xff) != data) {
            PRINTF(0, "[1seg] fc8150_if_btest!   i=0x%x, data=0x%x\n", i&0xff, data);
            res=1;
        }
    }


    for(i = 0 ; i < 1000 ; i++) {
        BBM_WORD_WRITE(0, 0xa4, i&0xffff);
        BBM_WORD_READ(0, 0xa4, &wdata);
        if((i & 0xffff) != wdata) {
            PRINTF(0, "[1seg] fc8150_if_wtest!   i=0x%x, data=0x%x\n", i&0xffff, wdata);
            res = 1;
        }
    }

    for(i = 0 ; i < 1000; i++) {
        BBM_LONG_WRITE(0, 0xa4, i&0xffffffff);
        BBM_LONG_READ(0, 0xa4, &ldata);
        if((i&0xffffffff) != ldata) {
            PRINTF(0, "[1seg] fc8150_if_ltest! i=0x%x, data=0x%x\n", i&0xffffffff, ldata);
            res=1;
        }
    }

    for(i=0 ; i < 1000 ; i++) {
        temp = i & 0xff;
        BBM_TUNER_WRITE(NULL, 0x52, 0x01, &temp, 0x01);
        BBM_TUNER_READ(NULL, 0x52, 0x01, &data, 0x01);
        if((i & 0xff) != data)
            PRINTF(0, "[1seg] FC8150 tuner test (0x%x,0x%x)\n", i & 0xff, data);
    }

    PRINTF(0, "[1seg] fc8150_if_test End!!!\n");

    return res;
}

static int isdbt_sw_lock_check(HANDLE hDevice)
{
    int res = BBM_NOK;
    unsigned char lock_data;

    res = BBM_READ(hDevice, 0x5053, &lock_data);
    PRINTF(0, "[1seg] isdbt_sw_lock_check lock : 0x%x, res : %d\n", lock_data, res);

    if(res)
        return res;

    if(lock_data & 0x01)
        res = BBM_OK;
    else
        res = BBM_NOK;

    return res;
}

void isdbt_get_signal_info(HANDLE hDevice,u16 *lock, u32 *ui32BER, u32 *ui32PER, s32 *rssi, u16 *cn, u16 *agc)
{
    struct dm_st {
        u8  start;
        s8  rssi;
        u8  wscn;
        u8  reserved;
        u16 main_rxd_rsps;
        u16 main_err_rsps;
        u32 main_err_bits;
        u32 dmp_rxd_bits;
        u32 dmp_err_bits;
        u16 inter_rxd_rsps;
        u16 inter_err_rsps;
        u32 inter_err_bits;
        u8  lna_code;
        u8  rfvga;
        u8  k;
        u8  csf_gain;
        u8  pga_gain;
        u8  extlna;
        u8  high_current_mode_gain;
        u8  extlna_gain;
    } dm;

    u8 bufIntStatus = 0;

    u8 agc_reg;

    if(isdbt_sw_lock_check(hDevice))
    {
        *lock = 0;
        *ui32BER = 10000;
        *ui32PER = 10000;
        *rssi = -100;
        *cn = 0;

        //return;
    }
    else
    {
        *lock = 1;
    }

    BBM_WRITE(hDevice, 0x5000, 0x0e);
    BBM_BULK_READ(hDevice, BBM_DM_DATA, (u8*) &dm + 1, sizeof(dm) - 1);

    BBM_READ(hDevice, BBM_BUF_STATUS, &bufIntStatus);

    BBM_READ(hDevice, 0x106e, &agc_reg);
    *agc = agc_reg;

    if(dm.inter_rxd_rsps)
        *ui32PER = ((dm.inter_err_rsps * 10000) / dm.inter_rxd_rsps);
    else
        *ui32PER = 10000;

    if(dm.dmp_rxd_bits)
        *ui32BER = ((dm.dmp_err_bits * 10000) / dm.dmp_rxd_bits);
    else
        *ui32BER = 10000;

    *rssi = dm.rssi;
    *cn = dm.wscn;

    PRINTF(hDevice, "[1seg][FC8150] LOCK :%d, BER: %d, PER : %d, RSSI : %d, CN : %d, agc : %d\n", *lock, *ui32BER, *ui32PER, *rssi, *cn, *agc);
    PRINTF(hDevice, "[1seg][FC8150] bufIntStatus :0x%x, BER: (%d/%d), PER : (%d/%d)\n", bufIntStatus, dm.dmp_err_bits, dm.dmp_rxd_bits, dm.inter_err_rsps, dm.inter_rxd_rsps);

}

void isdbt_set_scanmode(HANDLE hDevice, u8 scanmode)
{
    if(scanmode)
    {
        if(!scan_mode)
        {
            BBM_WRITE(hDevice, 0x3040, 0x00);
            BBM_WRITE(hDevice, 0x3004, 0x02);
            BBM_WRITE(hDevice, 0x3006, 0x02);
            BBM_WRITE(hDevice, 0x2020, 0x18);
            BBM_WRITE(hDevice, 0x2021, 0x14);
            BBM_WRITE(hDevice, 0x2022, 0xea);
            BBM_WRITE(hDevice, 0x2082, 0x70);
            BBM_WRITE(hDevice, 0x2083, 0x70);
            BBM_WRITE(hDevice, 0x2084, 0x70);
            BBM_WRITE(hDevice, 0x2085, 0x60);

            scan_mode=1;
            PRINTF(hDevice, "[1seg] SCAN MODE ON\n");
        }
    }
    else
    {
        if(scan_mode)
        {
            BBM_WRITE(hDevice, 0x3040, 0x27);
            BBM_WRITE(hDevice, 0x3004, 0x04);
            BBM_WRITE(hDevice, 0x3006, 0x04);
            BBM_WRITE(hDevice, 0x2020, 0x10);
            BBM_WRITE(hDevice, 0x2021, 0x0e);
            BBM_WRITE(hDevice, 0x2022, 0x4a);
            BBM_WRITE(hDevice, 0x2082, 0x45);
            BBM_WRITE(hDevice, 0x2083, 0x5f);
            BBM_WRITE(hDevice, 0x2084, 0x37);
            BBM_WRITE(hDevice, 0x2085, 0x30);

            scan_mode=0;
            PRINTF(hDevice, "[1seg] SCAN MODE OFF\n");
        }
    }

}

void isdbt_isr_check(HANDLE hDevice)
{
    u8 isr_time=0;

    BBM_WRITE(hDevice, BBM_BUF_INT, 0x00);

    while(isr_time < 10) {
        if(!isdbt_isr_sig) {
            break;
        }
        msWait(10);
        isr_time++;
    }

}

s32 isdbt_get_antenna_level(u32 ber, u16 CN, s32 prelvl)
{
    s32 antlvl;

    switch(prelvl)
    {
        case 0:
            if(ber < 650)
                antlvl = 1;
            else
                antlvl = prelvl;
        break;

        case 1:
            if((ber > 700) || ((ber > 500) && (CN <= 3)))
                antlvl = prelvl = 0;
            else if((ber < 300) && (CN > 6))
                antlvl = 2;
            else
                antlvl = prelvl;
        break;

        case 2:
            if((ber > 500) || ((ber > 300) && (CN <= 5)))
                antlvl = 1;
            else if((ber < 100) && (CN >= 9))
                antlvl = 3;
            else
                antlvl = prelvl;
        break;

        case 3:
            if((ber > 200) || ((ber > 100) && (CN <= 9)))
                antlvl = 2;
            else if((ber < 50) && (CN >= 12))
                antlvl = 4;
            else
                antlvl = prelvl;
        break;

        case 4:
            if((ber > 100) || (CN <= 14))
                antlvl = 3;
            else
                antlvl = prelvl;
        break;

        default :
            antlvl = 0;
        break;
    }

    return antlvl;
}
long isdbt_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    s32 res = BBM_NOK;

    void __user *argp = (void __user *)arg;

    s32 err = 0;
    s32 size = 0;
    int uData=0;
    ISDBT_OPEN_INFO_T *hOpen;

    IOCTL_ISDBT_SIGNAL_INFO isdbt_signal_info;

    if(_IOC_TYPE(cmd) != ISDBT_IOC_MAGIC)
    {
        return -EINVAL;
    }

    if(_IOC_NR(cmd) >= IOCTL_MAXNR)
    {
        return -EINVAL;
    }

    hOpen = filp->private_data;

    size = _IOC_SIZE(cmd);

    //PRINTF(0, "[1seg] isdbt_ioctl  0x%x\n", cmd);

    switch(cmd)
    {
        case IOCTL_ISDBT_POWER_ON:
        case LGE_BROADCAST_DMB_IOCTL_ON:
            PRINTF(0, "[1seg] IOCTL_ISDBT_POWER_ON \n");

            res = isdbt_hw_init();
            res |= BBM_I2C_INIT(hInit, FCI_I2C_TYPE);
            PRINTF(hInit, "[1seg] FC8150 BBM_I2C_INIT res : %d \n", res);

            res |= BBM_PROBE(hInit);
            PRINTF(hInit, "[1seg] FC8150 BBM_PROBE res : %d \n", res);

            if(res) {
                PRINTF(hInit, "[1seg] FC8150 Initialize Fail : %d \n", res);
            //    break;
            }

            res |= BBM_INIT(hInit);
            res |= BBM_TUNER_SELECT(hInit, FC8150_TUNER, 0);
            scan_mode = 0;

            if(res)
            PRINTF(0, "[1seg] IOCTL_ISDBT_POWER_ON FAIL \n");
            else
            PRINTF(0, "[1seg] IOCTL_ISDBT_POWER_OK \n");

            //fc8150_if_test();
            break;
        case IOCTL_ISDBT_POWER_OFF:
        case LGE_BROADCAST_DMB_IOCTL_OFF:

            PRINTF(0, "[1seg] IOCTL_ISDBT_POWER_OFF \n");
            isdbt_hw_deinit();
            res = BBM_OK;
            break;
        case IOCTL_ISDBT_SCAN_FREQ:
        {
            u32 f_rf;
            err = copy_from_user((void *)&uData, (void *)arg, size);
            #ifdef CONFIG_LGE_BROADCAST_BRAZIL_FREQ
            f_rf = (uData- 14) * 6000 + 473143;
            #else
            f_rf = (uData- 13) * 6000 + 473143;
            #endif
            PRINTF(0, "[1seg] IOCTL_ISDBT_SCAN_FREQ  f_rf : %d\n", f_rf);

            isdbt_set_scanmode(hInit, 1);

            isdbt_isr_check(hInit);
            res = BBM_TUNER_SET_FREQ(hInit, f_rf);
            BBM_WRITE(hInit, BBM_BUF_INT, 0x01);
            res |= BBM_SCAN_STATUS(hInit);
        }
            break;
        case IOCTL_ISDBT_SET_FREQ:
        {
            u32 f_rf;
            totalTS=0;
            totalErrTS=0;

            err = copy_from_user((void *)&uData, (void *)arg, size);
            mutex_lock(&ringbuffer_lock);
            fci_ringbuffer_flush(&hOpen->RingBuffer);
            mutex_unlock(&ringbuffer_lock);
            #ifdef CONFIG_LGE_BROADCAST_BRAZIL_FREQ
            f_rf = (uData- 14) * 6000 + 473143;
            #else
            f_rf = (uData- 13) * 6000 + 473143;
            #endif
            PRINTF(0, "[1seg] IOCTL_ISDBT_SET_FREQ chNum : %d, f_rf : %d\n", uData, f_rf);

            isdbt_set_scanmode(hInit, 0);
            isdbt_isr_check(hInit);
            res = BBM_TUNER_SET_FREQ(hInit, f_rf);
            BBM_WRITE(hInit, BBM_BUF_INT, 0x01);
            res |= BBM_SCAN_STATUS(hInit);
        }
            break;
        case IOCTL_ISDBT_GET_LOCK_STATUS:
            PRINTF(0, "[1seg] IOCTL_ISDBT_GET_LOCK_STATUS \n");
            res = isdbt_sw_lock_check(hInit);
            if(res)
                uData=0;
            else
                uData=1;
            err |= copy_to_user((void *)arg, (void *)&uData, size);
            res = BBM_OK;
            break;
        case IOCTL_ISDBT_GET_SIGNAL_INFO:
            isdbt_get_signal_info(hInit, &isdbt_signal_info.lock, &isdbt_signal_info.ber, &isdbt_signal_info.per, &isdbt_signal_info.rssi, &isdbt_signal_info.cn, &isdbt_signal_info.agc);

            isdbt_signal_info.ErrTSP = totalErrTS;
            isdbt_signal_info.TotalTSP = totalTS;

            totalTS=totalErrTS=0;

            err |= copy_to_user((void *)arg, (void *)&isdbt_signal_info, size);

            res = BBM_OK;

            break;
        case IOCTL_ISDBT_START_TS:
            hOpen->isdbttype = TS_TYPE;
            res = BBM_OK;
            break;
        case IOCTL_ISDBT_STOP_TS:
        case LGE_BROADCAST_DMB_IOCTL_USER_STOP:
            hOpen->isdbttype = 0;
            res = BBM_OK;
            break;

        case LGE_BROADCAST_DMB_IOCTL_SET_CH:
            {
                struct broadcast_dmb_set_ch_info udata;
                u32 f_rf;
                PRINTF(0, "[1seg] LGE_BROADCAST_DMB_IOCTL_SET_CH !!!X-tal : %d\n", bbm_xtal_freq);
                // disable irq
                mt65xx_eint_mask(CUST_EINT_DTV_NUM);

                if(copy_from_user(&udata, argp, sizeof(struct broadcast_dmb_set_ch_info)))
                {
                    PRINTF(0,"[1seg] broadcast_dmb_set_ch fail!!! \n");
                    res = -1;
                }
                else
                {
                #ifdef CONFIG_LGE_BROADCAST_BRAZIL_FREQ
                    f_rf = (udata.channel- 14) * 6000 + 473143;
                #else
                    f_rf = (udata.channel- 13) * 6000 + 473143;
                #endif
                    PRINTF(0, "[1seg] IOCTL_ISDBT_SET_FREQ freq:%d, RF:%d\n",udata.channel,f_rf);
                    if(udata.mode == LGE_BROADCAST_OPMODE_ENSQUERY)
                        isdbt_set_scanmode(hInit, 1);
                    else
                        isdbt_set_scanmode(hInit, 0);

                    isdbt_isr_check(hInit);
                    res = BBM_TUNER_SET_FREQ(hInit, f_rf);
                    if(res)
                        PRINTF(0, "[1seg] BBM_TUNER_SET_FREQ  Fail \n");
                    mt65xx_eint_unmask(CUST_EINT_DTV_NUM);
                    BBM_WRITE(hInit, BBM_BUF_INT, 0x01);

                    if(udata.mode == LGE_BROADCAST_OPMODE_ENSQUERY)
                    {
                        res |= BBM_SCAN_STATUS(hInit);
                        if(res != BBM_OK)
                        {
                            PRINTF(0, "[1seg] BBM_SCAN_STATUS  Unlock \n");
                            break;
                        }
                        PRINTF(0, "[1seg] BBM_SCAN_STATUS : Lock \n");
                    }

                    PRINTF(0, "[1seg] IOCTL_ISDBT_SET_FREQ \n");
                    totalTS=0;
                    totalErrTS=0;
                    ch_num = udata.channel;
                    mutex_lock(&ringbuffer_lock);
                    fci_ringbuffer_flush(&hOpen->RingBuffer);
                    mutex_unlock(&ringbuffer_lock);
                    hOpen->isdbttype = TS_TYPE;
                }
            }
            break;
        case LGE_BROADCAST_DMB_IOCTL_GET_SIG_INFO:
            {
                struct broadcast_dmb_sig_info udata;
                PRINTF(0, "[1seg] LGE_BROADCAST_DMB_IOCTL_GET_SIG_INFO \n");

                isdbt_get_signal_info(hInit, &isdbt_signal_info.lock, &isdbt_signal_info.ber, &isdbt_signal_info.per, &isdbt_signal_info.rssi, &isdbt_signal_info.cn, &isdbt_signal_info.agc);

                isdbt_signal_info.ErrTSP = totalErrTS;
                isdbt_signal_info.TotalTSP = totalTS;

                totalTS=totalErrTS=0;

                udata.info.oneseg_info.lock = (int)isdbt_signal_info.lock;
                udata.info.oneseg_info.ErrTSP = (int)isdbt_signal_info.ErrTSP;
                udata.info.oneseg_info.TotalTSP = (int)isdbt_signal_info.TotalTSP;

                udata.info.oneseg_info.ber = (int)isdbt_signal_info.ber;
                udata.info.oneseg_info.per = (int)isdbt_signal_info.per;
                udata.info.oneseg_info.rssi = (int)isdbt_signal_info.rssi;
                udata.info.oneseg_info.cn = (int)isdbt_signal_info.cn;
                udata.info.oneseg_info.agc = (int)isdbt_signal_info.agc;
                udata.info.oneseg_info.antenna_level = isdbt_get_antenna_level(isdbt_signal_info.ber, isdbt_signal_info.cn, udata.info.oneseg_info.antenna_level);

                if(copy_to_user((void *)argp, &udata, sizeof(struct broadcast_dmb_sig_info)))
                {
                    PRINTF(0,"[1seg] broadcast_dmb_get_sig_info copy_to_user error!!! \n");
                    res = BBM_NOK;
                }
                else
                {
                    /*PRINTF(0, "[1seg] LOCK :%d, BER: %d, PER : %d, RSSI : %d, CN : %d\n",
                        udata.info.oneseg_info.lock,
                        udata.info.oneseg_info.ber,
                        udata.info.oneseg_info.per,
                        udata.info.oneseg_info.rssi,
                        udata.info.oneseg_info.cn);*/

                    res = BBM_OK;
                }
            }
            break;

        case LGE_BROADCAST_DMB_IOCTL_GET_DMB_DATA:
            //                                                            
            res = ioctl_isdbt_read(hOpen,argp);
            break;
        case LGE_BROADCAST_DMB_IOCTL_OPEN:
        case LGE_BROADCAST_DMB_IOCTL_CLOSE:
        case LGE_BROADCAST_DMB_IOCTL_RESYNC:
        case LGE_BROADCAST_DMB_IOCTL_DETECT_SYNC:
        case LGE_BROADCAST_DMB_IOCTL_GET_CH_INFO:
        case LGE_BROADCAST_DMB_IOCTL_RESET_CH:
        case LGE_BROADCAST_DMB_IOCTL_SELECT_ANTENNA:
            PRINTF(0, "[1seg] LGE_BROADCAST_DMB_IOCTL_SKIP \n");
            res = BBM_OK;
            break;
        default:
            PRINTF(hInit, "[1seg] isdbt ioctl error!\n");
            res = BBM_NOK;
            break;
    }

    if(err < 0)
    {
        PRINTF(hInit, "copy to/from user fail : %d", err);
        res = BBM_NOK;
    }
    return res;
}

int isdbt_init(void)
{
    int ret = -1;
    int result = -1;
    s32 res;
    #if defined(CONFIG_ARCH_MT6572)// MediaTek platform
    result = g_lge_dtv_flag;
    PRINTF(hInit, "[dtv]g_lge_dtv_flag : [%d] result : [%d]\n", g_lge_dtv_flag, result);
    if(result == 1)
    {
        PRINTF(hInit, "[1seg] L50 DTV support model!!!\n");
    #endif defined(CONFIG_ARCH_MT6572)// MediaTek platform
    PRINTF(hInit, "[1seg] isdbt_init DRV V1p12 20140228\n");

    res = misc_register(&fc8150_misc_device);

    if(res < 0)
    {
        PRINTF(hInit, "[1seg] isdbt init fail : %d\n", res);
        return res;
    }

    wake_lock_init(&oneseg_wakelock, WAKE_LOCK_SUSPEND, fc8150_misc_device.name);

    isdbt_hw_setting();

    hInit = (ISDBT_INIT_INFO_T *)kmalloc(sizeof(ISDBT_INIT_INFO_T), GFP_KERNEL);

    res = BBM_HOSTIF_SELECT(hInit, BBM_SPI);

    if(res)
    {
        PRINTF(hInit, "[1seg] isdbt host interface select fail!\n");
    }

    if (!isdbt_kthread)
    {
        PRINTF(hInit, "[1seg] kthread run\n");
        isdbt_kthread = kthread_run(isdbt_thread, (void*)hInit, "isdbt_thread");
    }
    //irq setting
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
    // configuration for detect
    ret = (int)mt65xx_eint_set_sens(CUST_EINT_DTV_NUM, CUST_EINT_DTV_SENSITIVE);
    PRINTF(0,"[1seg][MTK] GPIO_1SEG_INT mt65xx_eint_set_sens = %d!!!\n", ret);
    mt65xx_eint_set_hw_debounce(CUST_EINT_DTV_NUM, CUST_EINT_DTV_DEBOUNCE_CN);
    PRINTF(0,"[1seg][MTK] GPIO_1SEG_INT mt65xx_eint_set_hw_debounce !!!\n");

    //gpio
    ret = mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_EINT);
    PRINTF(0,"[1seg][MTK] GPIO_1SEG_INT mt_set_gpio_mode = %d!!!\n", ret);
    ret = mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);
    PRINTF(0,"[1seg][MTK] GPIO_1SEG_INT mt_set_gpio_dir = %d!!!\n", ret);
    ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_DISABLE);
    PRINTF(0,"[1seg][MTK] GPIO_1SEG_INT mt_set_gpio_pull_disable = %d!!!\n", ret);

    // irq handler register
    mt65xx_eint_registration(CUST_EINT_DTV_NUM, CUST_EINT_DTV_DEBOUNCE_EN, CUST_EINT_POLARITY_HIGH/*CUST_EINT_DTV_POLARITY*/, isdbt_irq, 0);
    PRINTF(0,"[1seg][MTK] GPIO_1SEG_INT mt65xx_eint_registration!!!\n");

    // disable irq
    mt65xx_eint_mask(CUST_EINT_DTV_NUM);
#else //defined(CONFIG_ARCH_MT6572) // MediaTek platform
    res = request_irq(gpio_to_irq(GPIO_ISDBT_IRQ), isdbt_irq, IRQF_DISABLED | IRQF_TRIGGER_FALLING, FC8150_NAME, NULL);
    if(res)
        PRINTF(hInit, "[1seg] dmb rquest irq fail : %d\n", res);
#endif //defined(CONFIG_ARCH_MT6572) // MediaTek platform

#if defined(CONFIG_ARCH_MT6572)// MediaTek platform
    ret = mt_set_gpio_pull_enable(GPIO_1SEG_INT, GPIO_PULL_ENABLE);
    PRINTF(0,"[1seg][MTK] GPIO_1SEG_INT mt_set_gpio_pull_enable = %d!!!\n", ret);

    // Power OFF
    hwPowerDown(MT6323_POWER_LDO_VCAM_IO, "1V8_DTV");    // IO power Off
    PRINTF(0, "[1seg][MTK][init2] hwPowerDown MT6323_POWER_LDO_VCAM_IO\n");

    //SPI CS
    mt_set_gpio_mode(GPIO97, GPIO_MODE_00);  //GPIO_MATV_I2S_WS_PIN_M_GPIO  GPIO_MODE_00
      mt_set_gpio_pull_enable(GPIO97, GPIO_PULL_ENABLE);
      mt_set_gpio_pull_select(GPIO97,GPIO_PULL_DOWN);
      mt_set_gpio_dir(GPIO97, GPIO_DIR_OUT);
      mt_set_gpio_out(GPIO97, GPIO_OUT_ZERO);
    }
    else
    {
        PRINTF(hInit, "[1seg] DTV non-support model\n");
        return 0;
    }
#endif defined(CONFIG_ARCH_MT6572)// MediaTek platform
    INIT_LIST_HEAD(&(hInit->hHead));
    return 0;
}

void isdbt_exit(void)
{
    int ret = -1;
    PRINTF(hInit, "[1seg] isdbt isdbt_exit \n");
    // unregister irq
#if defined(CONFIG_ARCH_MT6572)    // MediaTek platform
    // gpio
    ret = mt_set_gpio_mode(GPIO_1SEG_INT, GPIO_1SEG_INT_M_GPIO);
    PRINTF(0,"[1seg][MTK][isdbt_exit] GPIO_1SEG_INT mt_set_gpio_mode = %d!!!\n", ret);
    ret = mt_set_gpio_dir(GPIO_1SEG_INT, GPIO_DIR_IN);
    PRINTF(0,"[1seg][MTK][isdbt_exit] GPIO_1SEG_INT mt_set_gpio_dir = %d!!!\n", ret);

    // mask
    //mt65xx_eint_mask(CUST_EINT_DTV_NUM);
#else //defined(CONFIG_ARCH_MT6572) // MediaTek platform
    free_irq(GPIO_ISDBT_IRQ, NULL);
#endif //defined(CONFIG_ARCH_MT6572) // MediaTek platform

    kthread_stop(isdbt_kthread);
    isdbt_kthread = NULL;

    BBM_HOSTIF_DESELECT(hInit);

    isdbt_hw_deinit();

    misc_deregister(&fc8150_misc_device);

    kfree(hInit);
    wake_lock_destroy(&oneseg_wakelock);
}

module_init(isdbt_init);
module_exit(isdbt_exit);

//MODULE_LICENSE("Dual BSD/GPL");
MODULE_LICENSE("GPL v2");
