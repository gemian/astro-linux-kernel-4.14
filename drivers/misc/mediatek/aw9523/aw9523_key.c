/**************************************************************************
*  aw9523_key.c
*
*  Create Date :
*
*  Modify Date :
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2016/02/15
**************************************************************************/

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/module.h>
//#include <linux/wakelock.h>
#include <linux/pm_wakeup.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/extcon.h>
#ifdef CONFIG_MTK_AW9524_UNDER_AW9523_LED
#include <mt-plat/mtk_pwm.h>
#include <linux/leds.h>
#endif

/**
 * General principals of this driver:
 * 1. Wait for any keypress with an interrupt, all 8 keyboard scan lines are monitored
 * 2. Check each keyboard scan line independently to determine combination of keys depressed
 * 3. Whilst keypresses are detected perform matrix scans to detect further key presses/releases
 * 4. When no keys are pressed return to interrupt based monitoring
 */

#define CONFIG_AW9523_HALL
#define AW9523_EARLY_SUSPEND

/**
 * AW9523_EARLY_SUSPEND added by wangyongsheng 20171227
 * Keys can be pressed whilst the screen is closed, early suspend provides a way disable the keyboard whilst the device
 * is closed, this can be triggered open/closed state (HALL) callback.
*/

#ifdef CONFIG_AW9523_HALL

#include <linux/notifier.h>
#include <soc/mediatek/hall.h>

#endif

#ifdef CONFIG_MTK_AW9524_UNDER_AW9523_LED

#define LED_MAX_BRIGHTNESS 5
#define DEFAULT_LED_NAME "kbd_backlight"

#endif

#include "aw9523_key.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AW9523_I2C_NAME "Integrated keyboard"

// These masks must have the lowest bits set and top bits clear.  No skipping rows.
#define P0_KROW_MASK 0xff
#define P1_KCOL_MASK 0x7f

#define KROW_P0_0 0
#define KROW_P0_1 1
#define KROW_P0_2 2
#define KROW_P0_3 3
#define KROW_P0_4 4
#define KROW_P0_5 5
#define KROW_P0_6 6
#define KROW_P0_7 7

#define KROW_P1_0 0
#define KROW_P1_1 1
#define KROW_P1_2 2
#define KROW_P1_3 3
#define KROW_P1_4 4
#define KROW_P1_5 5
#define KROW_P1_6 6
#define KROW_P1_7 7

//reg list
#define P0_INPUT     0x00 //Read for port input state
#define P1_INPUT     0x01 //Read for port input state
#define P0_OUTPUT    0x02 //R/W port output state
#define P1_OUTPUT    0x03 //R/W port output state
#define P0_CONFIG    0x04 //R/W port direction configuration
#define P1_CONFIG    0x05 //R/W port direction configuration
#define P0_INT       0x06 //R/W port interrupt enable
#define P1_INT       0x07 //R/W port interrupt enable
#define ID_REG       0x10 //Read for ID register
#define CTL_REG      0x11 //R/W global control register
#define P0_LED_MODE  0x12 //R/W LED Mode Switch
#define P1_LED_MODE  0x13 //R/W LED Mode Switch
#define P1_0_DIM0    0x20 //DIM0 LED current control
#define P1_1_DIM0    0x21 //DIM1
#define P1_2_DIM0    0x22 //DIM2
#define P1_3_DIM0    0x23 //DIM3
#define P0_0_DIM0    0x24 //DIM4
#define P0_1_DIM0    0x25 //DIM5
#define P0_2_DIM0    0x26 //DIM6
#define P0_3_DIM0    0x27 //DIM7
#define P0_4_DIM0    0x28 //DIM8
#define P0_5_DIM0    0x29 //DIM9
#define P0_6_DIM0    0x2A //DIM10
#define P0_7_DIM0    0x2B //DIM11
#define P1_4_DIM0    0x2C //DIM12
#define P1_5_DIM0    0x2D //DIM13
#define P1_6_DIM0    0x2E //DIM14
#define P1_7_DIM0    0x2F //DIM15
#define SW_RSTN      0x7F //Soft reset

#define MATRIX_SCAN_NANO_SECONDS_SLOW 10000000
#define MATRIX_SCAN_NANO_SECONDS_FAST 1000000

KEY_STATE key_map[] = {
//      name        code          val        row        col
        {"1",       KEY_1,          0, KROW_P0_0, KROW_P1_0},
        {"U",       KEY_U,          0, KROW_P0_1, KROW_P1_0},
        {"S",       KEY_S,          0, KROW_P0_2, KROW_P1_0},
        {"Z",       KEY_Z,          0, KROW_P0_3, KROW_P1_0},
        {",",       KEY_COMMA,      0, KROW_P0_4, KROW_P1_0},
        {"~",       KEY_APOSTROPHE, 0, KROW_P0_5, KROW_P1_0},
        {"8",       KEY_8,          0, KROW_P0_6, KROW_P1_0},
        {"J",       KEY_J,          0, KROW_P0_7, KROW_P1_0},

        {"2",       KEY_2,          0, KROW_P0_0, KROW_P1_1},
        {"W",       KEY_W,          0, KROW_P0_1, KROW_P1_1},
        {"D",       KEY_D,          0, KROW_P0_2, KROW_P1_1},
        {"C",       KEY_C,          0, KROW_P0_3, KROW_P1_1},
        {"ALT-L",   KEY_LEFTALT,    0, KROW_P0_4, KROW_P1_1},
        {"LEFT",    KEY_LEFT,       0, KROW_P0_5, KROW_P1_1},
        {"9",       KEY_9,          0, KROW_P0_6, KROW_P1_1},
        {"K",       KEY_K,          0, KROW_P0_7, KROW_P1_1},

        {"3",       KEY_3,          0, KROW_P0_0, KROW_P1_2},
        {"Y",       KEY_Y,          0, KROW_P0_1, KROW_P1_2},
        {"TAB",     KEY_TAB,        0, KROW_P0_2, KROW_P1_2},
        {"N",       KEY_N,          0, KROW_P0_3, KROW_P1_2},
        {"M",       KEY_M,          0, KROW_P0_4, KROW_P1_2},
        {"PGDN",    KEY_DOWN,       0, KROW_P0_5, KROW_P1_2},
        {"DEL",     KEY_BACKSPACE,  0, KROW_P0_6, KROW_P1_2},
        {"I",       KEY_I,          0, KROW_P0_7, KROW_P1_2},

        {"4",       KEY_4,          0, KROW_P0_0, KROW_P1_3},
        {"T",       KEY_T,          0, KROW_P0_1, KROW_P1_3},
        {"F",       KEY_F,          0, KROW_P0_2, KROW_P1_3},
        {"X",       KEY_X,          0, KROW_P0_3, KROW_P1_3},
        {"META-L",  KEY_LEFTMETA,   0, KROW_P0_4, KROW_P1_3},
        {"SHIFT-R", KEY_RIGHTSHIFT, 0, KROW_P0_5, KROW_P1_3},
        {"P",       KEY_P,          0, KROW_P0_6, KROW_P1_3},
        {"NULL",    KEY_UNKNOWN,    0, KROW_P0_7, KROW_P1_3},

        {"5",       KEY_5,          0, KROW_P0_0, KROW_P1_4},
        {"E",       KEY_E,          0, KROW_P0_1, KROW_P1_4},
        {"G",       KEY_G,          0, KROW_P0_2, KROW_P1_4},
        {"V",       KEY_V,          0, KROW_P0_3, KROW_P1_4},
        {"SPACE",   KEY_SPACE,      0, KROW_P0_4, KROW_P1_4},
        {"PGUP",    KEY_UP,         0, KROW_P0_5, KROW_P1_4},
        {"O",       KEY_O,          0, KROW_P0_6, KROW_P1_4},
        {"NULL",    KEY_UNKNOWN,    0, KROW_P0_7, KROW_P1_4},

        {"6",       KEY_6,          0, KROW_P0_0, KROW_P1_5},
        {"Q",       KEY_Q,          0, KROW_P0_1, KROW_P1_5},
        {"A",       KEY_A,          0, KROW_P0_2, KROW_P1_5},
        {"B",       KEY_B,          0, KROW_P0_3, KROW_P1_5},
        {"?",       KEY_DOT,        0, KROW_P0_4, KROW_P1_5},
        {"RIGHT",   KEY_RIGHT,      0, KROW_P0_5, KROW_P1_5},
        {"ENTER",   KEY_ENTER,      0, KROW_P0_6, KROW_P1_5},
        {"NULL",    KEY_UNKNOWN,    0, KROW_P0_7, KROW_P1_5},

        {"7",       KEY_7,          0, KROW_P0_0, KROW_P1_6},
        {"R",       KEY_R,          0, KROW_P0_1, KROW_P1_6},
        {"H",       KEY_H,          0, KROW_P0_2, KROW_P1_6},
        {"SHIFT-L", KEY_LEFTSHIFT,  0, KROW_P0_3, KROW_P1_6},
        {"CTRL",    KEY_LEFTCTRL,   0, KROW_P0_4, KROW_P1_6},
        {"L",       KEY_L,          0, KROW_P0_5, KROW_P1_6},
        {"0",       KEY_0,          0, KROW_P0_6, KROW_P1_6},
        {"NULL",    KEY_UNKNOWN,    0, KROW_P0_7, KROW_P1_6},
};

#define P0_NUM_MAX   8
#define P1_NUM_MAX   7
#define KEYST_MAX    2
#define KEYST_OLD    0
#define KEYST_NEW    1
static unsigned char keyst_old[P1_NUM_MAX];
static unsigned char keyst_new[P1_NUM_MAX];

static unsigned char keyst_def[KEYST_MAX][P1_NUM_MAX];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static unsigned char i2c_write_reg(unsigned char addr, unsigned char reg_data);
static unsigned char i2c_read_reg(unsigned char addr);

static ssize_t aw9523_get_reg(struct device *cd, struct device_attribute *attr, char *buf);
static ssize_t aw9523_set_reg(struct device *cd, struct device_attribute *attr, const char *buf, size_t len);

static DEVICE_ATTR(reg, 0660, aw9523_get_reg, aw9523_set_reg);

struct aw9523_key_data {
    struct device *dev;
    struct input_dev *input_dev;
    struct work_struct eint_work;
    struct device_node *irq_node;
    struct hrtimer key_timer;
    int irq;
    bool irq_enabled;
    struct delayed_work work;
    int delay;
    KEY_STATE *keymap;
    int keymap_len;
#ifdef CONFIG_AW9523_HALL
    struct notifier_block hall_notif;
    bool is_device_closed;
#endif
#ifdef CONFIG_MTK_AW9524_UNDER_AW9523_LED
	struct led_classdev led_dev;
#endif
};

struct aw9523_pinctrl {
    struct pinctrl *pinctrl;
    struct pinctrl_state *shdn_high;
    struct pinctrl_state *shdn_low;
    struct pinctrl_state *int_pin;
};

struct pinctrl *aw9523_pin; //CPU IO pin connected to AW9523 RSTN Hardware reset
struct pinctrl_state *shdn_high;
struct pinctrl_state *shdn_low;
struct pinctrl_state *int_pin;
struct aw9523_key_data *aw9523_key;
struct i2c_client *aw9523_i2c_client;

#ifdef CONFIG_AW9523_HALL
static int aw9523_hall_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

#ifdef AW9523_EARLY_SUSPEND
static void aw9523_i2c_early_suspend(struct i2c_client *client);
static void aw9523_i2c_early_resume(struct i2c_client *client);
#endif

bool aw9523MetaKeyPressed = false;
struct input_dev *aw9523_kpd_input_dev;
extern struct extcon_dev *fcover_data;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIO Control
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int aw9523_pinctrl_init(struct platform_device *pdev) {
    int ret = 0;
    aw9523_pin = devm_pinctrl_get(&pdev->dev);
    AW9523_LOG("%s : pinctrl init 00000000\n", __func__);
    if (IS_ERR(aw9523_pin)) {
        dev_err(&pdev->dev, "Cannot find aw9523 pinctrl!");
        ret = PTR_ERR(aw9523_pin);
        pr_err("%s devm_pinctrl_get fail!\n", __func__);
    }

    AW9523_LOG("%s : pinctrl init 11111111\n", __func__);
    shdn_high = pinctrl_lookup_state(aw9523_pin, "aw9523_shdn_high");
    if (IS_ERR(shdn_high)) {
        ret = PTR_ERR(shdn_high);
        pr_err("%s : pinctrl err, aw9523_shdn_high\n", __func__);
    }

    shdn_low = pinctrl_lookup_state(aw9523_pin, "aw9523_shdn_low");
    if (IS_ERR(shdn_low)) {
        ret = PTR_ERR(shdn_low);
        pr_err("%s : pinctrl err, aw9523_shdn_low\n", __func__);
    }

    return ret;
}

static void aw9523_hw_reset(void) {
    AW9523_LOG("%s enter\n", __func__);
    pinctrl_select_state(aw9523_pin, shdn_low);
    msleep(5);
    pinctrl_select_state(aw9523_pin, shdn_high);
    msleep(5);
    AW9523_LOG("%s out\n", __func__);
}

static void aw9523_reset_to_monitor_for_state_change(void) {
    unsigned char val;

    val = i2c_read_reg(P1_CONFIG);
    i2c_write_reg(P1_CONFIG, val & (~P1_KCOL_MASK));    //set p1 port output mode

    val = i2c_read_reg(P1_OUTPUT);
    i2c_write_reg(P1_OUTPUT, val & (~P1_KCOL_MASK));    //p1 port output 0

    val = i2c_read_reg(P0_INPUT);    //clear p0 input irq

    val = i2c_read_reg(P0_INT);
    i2c_write_reg(P0_INT, 0x00);    //enable p0 port irq

    AW9523_LOG("%s irq enabled: %d\n", __func__, aw9523_key->irq_enabled);
//    if (!aw9523_key->irq_enabled) {
    enable_irq(aw9523_key->irq);
    aw9523_key->irq_enabled = true;
//    }
}

static void aw9523_schedule_matrix_rescan(bool fast) {
    int nanoSeconds = fast ? MATRIX_SCAN_NANO_SECONDS_FAST : MATRIX_SCAN_NANO_SECONDS_SLOW;
    hrtimer_start(&aw9523_key->key_timer, ktime_set(0, nanoSeconds), HRTIMER_MODE_REL);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void aw9523_key_eint_work(struct work_struct *work) {
    KEY_STATE *keymap;
    unsigned char i, j, idx;
    unsigned char val;
    bool update_now = false; // Used to detect ghosting
    int keymap_len;
    int x = 0;
    int y = 0;
    int t;
    // These data structures are large enough they can't overflow.
    int press_count = 0;
    int press_codes[P0_NUM_MAX * P1_NUM_MAX];
    int release_count = 0;
    int release_codes[P0_NUM_MAX * P1_NUM_MAX];
    bool setKeyValue;
    bool forceAllKeyRelease;

    AW9523_LOG("Handling Interrupt (%d)\n",aw9523_key->irq_enabled);

#ifdef CONFIG_AW9523_HALL
    if (aw9523_key->is_device_closed) {
        AW9523_LOG("Device is closed\n");

        aw9523_reset_to_monitor_for_state_change();
        return;
    }
#endif

    keymap = aw9523_key->keymap;
    keymap_len = aw9523_key->keymap_len;

    for (i = 0; i < P1_NUM_MAX; i++) {
        if (P1_KCOL_MASK & (1 << i)) {
            val = i2c_read_reg(P1_CONFIG);
            i2c_write_reg(P1_CONFIG, (P1_KCOL_MASK | val) & (~(1 << i)));    //set p1_x port output mode
            val = i2c_read_reg(P1_OUTPUT);
            i2c_write_reg(P1_OUTPUT, (P1_KCOL_MASK | val) & (~(1 << i)));

            val = i2c_read_reg(P0_INPUT);    // read p0 port status

            setKeyValue = true;
#ifdef CONFIG_AW9523_HALL
            setKeyValue = !aw9523_key->is_device_closed;
#endif
            if (setKeyValue) {
                keyst_new[i] = (val & P0_KROW_MASK);
            }
            //AW9523_LOG("0x%02x, ", keyst_new[i]);                //i=p1 keyst[i]=p0
        }
    }
    // AW9523_LOG("\n");


    /* This routine prevents ghosting.  As an example, if Control+L_Shift+N is pressed,
     * the keyboard detects both N & M at the same time due to the electronic circuit.
     * Rather than detect both keys, block both keys until either Control or L_Shift is
     * released, then detect the correct keypress. See youtu.be/L3ByBtM-w9I */
    if (memcmp(keyst_old, keyst_new, P1_NUM_MAX)) {    // keyst changed

        // Special cases for ctrl+shift with z/x/c/v clear unlikely combination
        if (((keyst_new[KROW_P1_6] & (1 << KROW_P0_3)) == 0) && //l_shift
            ((keyst_new[KROW_P1_6] & (1 << KROW_P0_4)) == 0)) { //ctrl
            if ((keyst_new[KROW_P1_0] & (1 << KROW_P0_3)) == 0) { //z
                keyst_new[KROW_P1_0] |= (1 << KROW_P0_4); // -/
            }
            if ((keyst_new[KROW_P1_3] & (1 << KROW_P0_3)) == 0) { //x
                keyst_new[KROW_P1_3] |= (1 << KROW_P0_4); // -fn
            }
            if ((keyst_new[KROW_P1_1] & (1 << KROW_P0_3)) == 0) { //c
                keyst_new[KROW_P1_1] |= (1 << KROW_P0_4); // -alt
            }
            if ((keyst_new[KROW_P1_4] & (1 << KROW_P0_3)) == 0) { //v
                keyst_new[KROW_P1_4] |= (1 << KROW_P0_4); // -space
            }
        }

        val = P0_KROW_MASK; // The distinct columns used
        y = 0;
        update_now = true; // Now by default, do the update since keyst changed.
        for (i = 0; i < P1_NUM_MAX; i++) {
            if (keyst_new[i] == P0_KROW_MASK) {
                // Most of the time no key is pressed, skip it.
                continue;
            }
            x = 0; // Count the number of keys pressed (ie. clear bits)
            for (j = 0; j < P0_NUM_MAX; j++) {
                if (!(keyst_new[i] & (1 << j))) {    // press
                    x++; // Increase whenever this bit is clear.
                    // Boolean expression is false if a previous row
                    // also has this bit clear.  In this case, the
                    // keyboard is ghosting, and update_now is false.
                    update_now = update_now && (val & (1 << j));
                }
            }
            // If more than one key in this row is pressed, remember which bits
            // are clear so we can check for ghosting in other rows.
            if (x > 1) {
                val &= keyst_new[i];
            }
        }
    }

    // update_now is set when the key state changes and there's no ghosting right now.
    if (update_now) {
        for (t = 0; t < P1_NUM_MAX; t++) {
            i = t;
            // I'm not sure why these are swapped.  Is it still needed?
            if (t == 3)
                i = 6;
            if (t == 6)
                i = 3;
            if (keyst_old[i] == keyst_new[i]) {
                continue; // Skip if keyst of i didn't change.
            }
            for (j = 0; j < P0_NUM_MAX; j++) {
                if ((keyst_old[i] & (1 << j)) != (keyst_new[i] & (1 << j))) {    // j row & i col changed
                    // The keymap data structure is organised this way.
                    idx = i * P0_NUM_MAX + j;
                    if (keyst_new[i] & (1 << j)) {    // release
                        keymap[idx].key_val = 0;
                        release_codes[release_count] = keymap[idx].key_code;
                        release_count++;
                        if (keymap[idx].key_code == KEY_LEFTMETA) {
                            AW9523_LOG("LEFTMETA RELEASE");
                            aw9523MetaKeyPressed = false;
                        }
                    } else {    // press
                        keymap[idx].key_val = 1;
                        press_codes[press_count] = keymap[idx].key_code;
                        press_count++;
                        if (keymap[idx].key_code == KEY_LEFTMETA) {
                            AW9523_LOG("LEFTMETA PRESS");
                            aw9523MetaKeyPressed = true;
                        }
                    }
                    AW9523_LOG("Storing key code %d val %d\n",
                               keymap[idx].key_code,
                               keymap[idx].key_val);
                }
            }
        }

        /* Process key presses before releases because sometimes a release of one key will
         * unblock another key press, and we want to keep all modifier keys pressed. */
        for (t = 0; t < press_count; t++) {
            AW9523_LOG("Processing key press in position %d code %d\n",
                       t, press_codes[t]);
            input_report_key(aw9523_key->input_dev,
                             press_codes[t],
                             1); // The one records a press
        }
        // Now go through all the releases.
        for (t = 0; t < release_count; t++) {
            AW9523_LOG("Processing key release in position %d code %d\n",
                       t, release_codes[t]);
            input_report_key(aw9523_key->input_dev,
                             release_codes[t],
                             0); // The zero records a release
        }
        input_sync(aw9523_key->input_dev);

        // Store the current state so we can detect a change next time.
        memcpy(keyst_old, keyst_new, P1_NUM_MAX);
    }

    forceAllKeyRelease = false;
#ifdef CONFIG_AW9523_HALL
    forceAllKeyRelease = aw9523_key->is_device_closed;
#endif

    if (((!(memcmp(&keyst_new[0], &keyst_def[KEYST_NEW][0], P1_NUM_MAX)))) || forceAllKeyRelease) {
        AW9523_LOG("No keys pressed return to interrupt monitoring\n");
        aw9523_reset_to_monitor_for_state_change();
    } else {
        AW9523_LOG("Scheduling matrix rescan - keys held down\n");
        aw9523_schedule_matrix_rescan(false);
    }
    AW9523_LOG("%s: end \n", __func__);
}

static enum hrtimer_restart aw9523_key_timer_func(struct hrtimer *timer)
{
	AW9523_LOG("HRTimer\n");

	schedule_work(&aw9523_key->eint_work);

	return HRTIMER_NORESTART;
}

/*********************************************************
 *
 * int work
 *
 ********************************************************/
static void aw9523_int_work(struct work_struct *work) {

    AW9523_LOG("DelayedWork\n");

    i2c_write_reg(P0_INT, 0xff);    //disable p0 port irq
    i2c_read_reg(P0_INPUT);    // clear P0 Input Interrupt



    aw9523_schedule_matrix_rescan(true);
}

static irqreturn_t aw9523_key_eint_func(int irq, void *desc) {
    AW9523_LOG("IRQ disable I nosync\n");
    disable_irq_nosync(aw9523_key->irq);
    aw9523_key->irq_enabled = false;
    AW9523_LOG("%s irq enabled: %d (Int Enter)\n", __func__, aw9523_key->irq_enabled);

    if (aw9523_key == NULL) {
        AW9523_LOG("aw9523_key == NULL");
        return IRQ_NONE;
    }
    schedule_delayed_work(&aw9523_key->work, msecs_to_jiffies(1));

    return IRQ_HANDLED;

}

int aw9523_key_setup_eint(void) {
    int ret = 0;
    u32 ints[2] = {0, 0};

    int_pin = pinctrl_lookup_state(aw9523_pin, "aw9523_int_pin");
    if (IS_ERR(int_pin)) {
        ret = PTR_ERR(int_pin);
        pr_debug("%s : pinctrl err, aw9523_int_pin\n", __func__);
    }

    aw9523_key->irq_node = of_find_compatible_node(NULL, NULL, "mediatek,aw9523_eint");

    if (aw9523_key->irq_node) {
        of_property_read_u32_array(aw9523_key->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);
        pinctrl_select_state(aw9523_pin, int_pin);
        AW9523_LOG("%s ints[0] = %d, ints[1] = %d!!\n", __func__, ints[0], ints[1]);

        aw9523_key->irq = irq_of_parse_and_map(aw9523_key->irq_node, 0);
        AW9523_LOG("%s irq = %d\n", __func__, aw9523_key->irq);
        if (!aw9523_key->irq) {
            pr_err("%s irq_of_parse_and_map fail!!\n", __func__);
            return -EINVAL;
        }
        if (request_irq (aw9523_key->irq, aw9523_key_eint_func, IRQ_TYPE_NONE, "aw9523-eint", NULL)) {
            pr_err("%s IRQ LINE NOT AVAILABLE!!\n", __func__);
            return -EINVAL;
        }
        aw9523_key->irq_enabled = true;
        AW9523_LOG("%s irq enabled: %d\n", __func__, aw9523_key->irq_enabled);
    } else {
        AW9523_LOG("null irq node!!\n");
        return -EINVAL;
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static unsigned char i2c_write_reg(unsigned char addr, unsigned char reg_data) {
    char ret;
    u8 wdbuf[512] = {0};

    struct i2c_msg msgs[] = {
            {
                    .addr = aw9523_i2c_client->addr,
                    .flags = 0,
                    .len = 2,
                    .buf = wdbuf,
            },
    };

    wdbuf[0] = addr;
    wdbuf[1] = reg_data;

    ret = i2c_transfer(aw9523_i2c_client->adapter, msgs, 1);
    if (ret < 0)
        pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

static unsigned char i2c_read_reg(unsigned char addr) {
    unsigned char ret;
    u8 rdbuf[512] = {0};

    struct i2c_msg msgs[] = {
            {
                    .addr = aw9523_i2c_client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = rdbuf,
            },
            {
                    .addr = aw9523_i2c_client->addr,
                    .flags = I2C_M_RD,
                    .len = 1,
                    .buf = rdbuf,
            },
    };

    rdbuf[0] = addr;

    ret = i2c_transfer(aw9523_i2c_client->adapter, msgs, 2);
    if (ret < 0)
        pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// aw9523 init
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void aw9523_init_keycfg(void) {
    i2c_write_reg(SW_RSTN, 0x00);    // Software Reset

#ifdef CONFIG_MTK_AW9524_UNDER_AW9523_LED
    i2c_write_reg(P1_LED_MODE, 0x7F); //p1-7 led mode
    i2c_write_reg(P1_7_DIM0, 0x00);
#endif

    i2c_write_reg(P0_CONFIG, 0xFF);    // P0: Input Mode
    i2c_write_reg(P1_CONFIG, 0x00);    // P1: Output Mode
    i2c_write_reg(P1_OUTPUT, 0x00);    // P1: 0000 0000

    i2c_write_reg(P0_INT, 0x00);    // P0: Enable Interrupt
    i2c_write_reg(P1_INT, 0xFF);    // P1: Disable Interrupt
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Debug
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw9523_get_reg(struct device *cd, struct device_attribute *attr, char *buf) {
    unsigned char reg_val;
    ssize_t len = 0;
    u8 i;
    for (i = 0; i < 0x30; i++) {
        reg_val = i2c_read_reg(i);
        len += snprintf(buf + len, PAGE_SIZE - len, "reg%2X = 0x%2X, ", i, reg_val);
    }

    return len;
}

static ssize_t aw9523_set_reg(struct device *cd, struct device_attribute *attr, const char *buf, size_t len) {
    unsigned int databuf[2];
    if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        i2c_write_reg(databuf[0], databuf[1]);
    }
    return len;
}

static int aw9523_create_sysfs(struct i2c_client *client) {
    int err;
    struct device *dev = &(client->dev);

    err = device_create_file(dev, &dev_attr_reg);

    return err;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void aw9523_input_register(void) {
    int err;
	aw9523_kpd_input_dev = input_allocate_device();
    if (!aw9523_kpd_input_dev) {
        AW9523_LOG("No kpd_input_dev");
        err = -ENOMEM;
        goto exit_input_dev_alloc_failed;
    }
    AW9523_LOG("%s[%d]\n", __func__,__LINE__);
    aw9523_key->input_dev = aw9523_kpd_input_dev;
    __set_bit(EV_KEY, aw9523_kpd_input_dev->evbit);
    __set_bit(EV_SYN, aw9523_kpd_input_dev->evbit);

    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_ESC);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_POWER);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_1);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_2);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_3);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_4);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_5);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_6);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_7);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_8);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_9);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_0);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_U);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_W);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_Y);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_T);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_E);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_Q);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_R);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_S);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_D);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_TAB);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_F);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_G);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_A);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_H);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_Z);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_C);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_N);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_X);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_V);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_B);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_LEFTSHIFT);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_COMMA);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_LEFTALT);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_M);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_LEFTMETA);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_SPACE);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_DOT);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_LEFTCTRL);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_APOSTROPHE);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_LEFT);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_DOWN);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_RIGHTSHIFT);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_UP);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_RIGHT);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_L);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_COMMA);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_BACKSPACE);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_P);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_O);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_ENTER);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_J);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_K);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_I);
    input_set_capability(aw9523_key->input_dev, EV_KEY, KEY_UNKNOWN);

    aw9523_kpd_input_dev->name = AW9523_I2C_NAME;
    err = input_register_device(aw9523_kpd_input_dev);
    if (err) {
        AW9523_LOG("aw9523_input_register: failed to register input device\n");
        goto exit_input_register_device_failed;
    }
    return;

exit_input_dev_alloc_failed:
    cancel_work_sync(&aw9523_key->eint_work);
exit_input_register_device_failed:
    input_free_device(aw9523_kpd_input_dev);
}

#ifdef CONFIG_AW9523_HALL
static int aw9523_hall_notifier_callback(struct notifier_block *self, unsigned long event, void *data) {
    struct aw9523_key_data *aw9523 = container_of(self, struct aw9523_key_data, hall_notif);
    // Must indicate device is closed before suspending, and resume before setting device opened.
    // This is to avoid reporting random keys during suspend/resume
    if (event == HALL_FCOVER_CLOSE) {
        AW9523_LOG("%s: hall notify device closed.\n", __func__);
        aw9523->is_device_closed = true;
        aw9523_i2c_early_suspend(aw9523_i2c_client);
    } else if (event == HALL_FCOVER_OPEN) {
        AW9523_LOG("%s: hall notify device opened.\n", __func__);
        aw9523_i2c_early_resume(aw9523_i2c_client);
        aw9523->is_device_closed = false;
    }
    return 0;
}
#endif

#ifdef CONFIG_MTK_AW9524_UNDER_AW9523_LED

static void gpio_main_pwm_keyboardlight(u32 data1, u32 data2)
{
    struct pwm_spec_config pwm_setting;
    pwm_setting.pwm_no	= PWM1;
    pwm_setting.mode	= PWM_MODE_FIFO;
    pwm_setting.clk_div = CLK_DIV8;
    pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
    pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = false;
    pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
    pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
    pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
    pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
    pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
    pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
    pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = data1;
    pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = data2;
    pwm_set_spec_config(&pwm_setting);
}

static ssize_t aw9524_led_proc_fops_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    char acBuf[6];
    char *str_buf = acBuf;
	unsigned char reg_val_1;
	unsigned char brightness=10;
    if (count >= sizeof(acBuf)){
        pr_warn("%s() len outsize!! %ld\n", __func__, count);
        return -EFAULT;
    }
    if (copy_from_user(str_buf, buff, count) ){
        pr_warn("copy_from_user---error\n");
        return -EFAULT;
    }
	reg_val_1 = i2c_read_reg(P1_OUTPUT);
	AW9523_LOG("start P1_OUTPUT_AW9524 reg_val_1 = 0x%2X\n", reg_val_1);
	AW9523_LOG("start  str_buf = 0x%d\n", str_buf[2]);
	if (str_buf[0]== '1') {
		if (str_buf[1]== '3') {
            i2c_write_reg(P1_7_DIM0,(((int)str_buf[2]-48)*brightness));
		}
	} else if (str_buf[0]== '4'){  //keyboard light
		if (str_buf[1]== '0') {
			gpio_main_pwm_keyboardlight(0x00000000, 0x00000000); //0
		} else if (str_buf[1]== '1') {
			gpio_main_pwm_keyboardlight(0x00001fff, 0x00000000); //20
		} else if (str_buf[1]== '2') {
			gpio_main_pwm_keyboardlight(0x07ffffff, 0x00000000); //40
		} else if (str_buf[1]== '3') {
			gpio_main_pwm_keyboardlight(0xffffffff, 0x0000003F); //60
		}else if (str_buf[1]== '4') {
			gpio_main_pwm_keyboardlight(0xffffffff, 0x000fffff); //80
		}else if (str_buf[1]== '5') {
			gpio_main_pwm_keyboardlight(0xffffffff, 0xffffffff); //100
		}
	}else{
		AW9523_LOG("proc end!\n");
	}
	reg_val_1 = i2c_read_reg(P1_OUTPUT);
	AW9523_LOG("end P1_OUTPUT_AW9524 reg_val_1 = 0x%2X\n", reg_val_1);
    AW9523_LOG("%s \n", str_buf);
    return count;
}

static const struct file_operations aw9524_led_proc_fops = {
	.write = aw9524_led_proc_fops_write
};

static void aw9524_brightness_set(struct led_classdev *led_cdev,
								  enum led_brightness val)
{
	switch ((int)val) {
		case 0:
			gpio_main_pwm_keyboardlight(0x00000000, 0x00000000); //0
			break;
		case 1:
			gpio_main_pwm_keyboardlight(0x00001fff, 0x00000000); //20
			break;
		case 2:
			gpio_main_pwm_keyboardlight(0x07ffffff, 0x00000000); //40
			break;
		case 3:
			gpio_main_pwm_keyboardlight(0xffffffff, 0x0000003F); //60
			break;
		case 4:
			gpio_main_pwm_keyboardlight(0xffffffff, 0x000fffff); //80
			break;
		case 5:
			gpio_main_pwm_keyboardlight(0xffffffff, 0xffffffff); //100
			break;
	}
}


#endif

static int aw9523_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    unsigned char reg_value = 0;
    int err = 0;
    unsigned char cnt = 5;
    AW9523_LOG("%s start - addr: %d\n", __func__, client->addr);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }
    AW9523_LOG("%s: kzalloc\n", __func__);
    aw9523_key = kzalloc(sizeof(*aw9523_key), GFP_KERNEL);
    if (!aw9523_key) {
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    aw9523_i2c_client = client;
    i2c_set_clientdata(client, aw9523_key);

    aw9523_hw_reset();

    // CHIP ID
    while ((cnt > 0) && (reg_value != 0x23)) {
        reg_value = i2c_read_reg(0x10);
        AW9523_LOG("aw9523 chipid=0x%2x\n", reg_value);
        cnt--;
        msleep(10);
    }
    if (!cnt) {
        err = -ENODEV;
        goto exit_create_singlethread;
    }

    INIT_DELAYED_WORK(&aw9523_key->work, aw9523_int_work);

#ifdef CONFIG_MTK_AW9524_UNDER_AW9523_LED
    proc_create("aw9524_led_proc", 0777, NULL, &aw9524_led_proc_fops);
	AW9523_LOG("aw9524_led_proc\n");

	aw9523_key->led_dev.max_brightness = LED_MAX_BRIGHTNESS;
	aw9523_key->led_dev.brightness_set = aw9524_brightness_set;
	aw9523_key->led_dev.name = DEFAULT_LED_NAME;

	err = devm_led_classdev_register(&client->dev, &aw9523_key->led_dev);
	if (err) {
		AW9523_LOG("led register err: %d\n", ret);
		goto exit_create_singlethread;
	}
#endif

    aw9523_key->delay = 10;    //50
    aw9523_key->dev = &client->dev;

    aw9523_input_register();

    aw9523_key->keymap_len = sizeof(key_map) / sizeof(KEY_STATE);
    aw9523_key->keymap = (KEY_STATE *) &key_map;

    aw9523_init_keycfg();

#ifdef CONFIG_AW9523_HALL
    aw9523_key->is_device_closed = true;
    if (fcover_data) {
        aw9523_key->is_device_closed = (extcon_get_state(fcover_data, EXTCON_MECHANICAL) == HALL_FCOVER_CLOSE);
    }
    aw9523_key->hall_notif.notifier_call = aw9523_hall_notifier_callback;
    err = hall_register_client(&aw9523_key->hall_notif);
    if (err) {
        AW9523_LOG("Unable to register aw9523_key fb_notifier: %d\n", err);
    } else {
        AW9523_LOG("Success to register aw9523_key fb_notifier.\n");
    }
#endif

    //Interrupt
    aw9523_key_setup_eint();
    INIT_WORK(&aw9523_key->eint_work, aw9523_key_eint_work);
    hrtimer_init(&aw9523_key->key_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw9523_key->key_timer.function = aw9523_key_timer_func;

    aw9523_create_sysfs(client);

    memset(keyst_new, P0_KROW_MASK, sizeof(keyst_new));
    memset(keyst_old, P0_KROW_MASK, sizeof(keyst_old));
    memset(keyst_def, P0_KROW_MASK, sizeof(keyst_def));
	AW9523_LOG("probe ok!!\n");
    return 0;

    exit_create_singlethread:
    aw9523_i2c_client = NULL;
    exit_alloc_data_failed:
    kfree(aw9523_key);
    exit_check_functionality_failed:
    return err;
}

#define AW9523_I2C_SUSPEND

#ifdef AW9523_EARLY_SUSPEND

static void aw9523_i2c_early_suspend(struct i2c_client *client) {
    struct aw9523_key_data *aw9523_key = i2c_get_clientdata(client);

    AW9523_LOG("IRQ disable es 1 nosync\n");
    disable_irq_nosync(aw9523_key->irq);
    aw9523_key->irq_enabled = false;

    pinctrl_select_state(aw9523_pin, shdn_low);
    msleep(5);
    AW9523_LOG("%s enter111!\n", __func__);

    return;
}

/*----------------------------------------------------------------------------*/
static void aw9523_i2c_early_resume(struct i2c_client *client) {
    AW9523_LOG("%s enter\n", __func__);

    aw9523_hw_reset();
    aw9523_init_keycfg();

    AW9523_LOG("Scheduling matrix rescan - resume check\n");
    aw9523_schedule_matrix_rescan(false);
    return;
}

#endif

#ifdef AW9523_I2C_SUSPEND

static int aw9523_i2c_suspend(struct i2c_client *client, pm_message_t msg) {
#ifndef AW9523_EARLY_SUSPEND
    struct aw9523_key_data *aw9523_key = i2c_get_clientdata(client);

    AW9523_LOG("IRQ disable S nosync\n");
    disable_irq_nosync(aw9523_key->irq);
    aw9523_key->irq_enabled = false;

    if (msg.event == PM_EVENT_SUSPEND) {
        pinctrl_select_state(aw9523_pin, shdn_low);
        msleep(5);
        AW9523_LOG("%s enter111!\n", __func__);
        cancel_delayed_work_sync(&aw9523_key->work);
        AW9523_LOG("%s enter222!\n", __func__);
        cancel_work_sync(&aw9523_key->eint_work);
    }
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
static int aw9523_i2c_resume(struct i2c_client *client) {
#ifndef AW9523_EARLY_SUSPEND
    struct aw9523_key_data *aw9523_key = i2c_get_clientdata(client);

    AW9523_LOG("IRQ Re-enable 2 resume\n");
    AW9523_LOG("%s irq enabled: %d\n", __func__, aw9523_key->irq_enabled);
    if (!aw9523_key->irq_enabled) {
        enable_irq(aw9523_key->irq);
        aw9523_key->irq_enabled = true;
    }

    aw9523_hw_reset();
    aw9523_init_keycfg();
    INIT_DELAYED_WORK(&aw9523_key->work, aw9523_int_work);
    INIT_WORK(&aw9523_key->eint_work, aw9523_key_eint_work);
#endif
    return 0;
}

#endif

static int aw9523_i2c_remove(struct i2c_client *client) {
    struct aw9523_key_data *aw9523_key = i2c_get_clientdata(client);

    AW9523_LOG("%s enter\n", __func__);

    cancel_delayed_work_sync(&aw9523_key->work);
    cancel_work_sync(&aw9523_key->eint_work);
    input_unregister_device(aw9523_key->input_dev);

    kfree(aw9523_key);

    aw9523_i2c_client = NULL;
    i2c_set_clientdata(client, NULL);
#ifdef CONFIG_AW9523_HALL
    hall_unregister_client(&aw9523_key->hall_notif);
#endif

    return 0;
}

static const struct i2c_device_id aw9523_i2c_id[] = {
    {AW9523_I2C_NAME, 0},
    {}
};

#ifdef CONFIG_OF
static const struct of_device_id extgpio_of_match[] = {
    {.compatible = "mediatek,aw9523_key"},
    {},
};
#endif

static struct i2c_driver aw9523_i2c_driver = {
    .driver = {
        .name = AW9523_I2C_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = extgpio_of_match,
#endif
    },

    .probe = aw9523_i2c_probe,
    .remove = aw9523_i2c_remove,
#ifdef AW9523_I2C_SUSPEND
    .suspend = aw9523_i2c_suspend,
    .resume = aw9523_i2c_resume,
#endif
    .id_table = aw9523_i2c_id,
};


static int aw9523_key_remove(struct platform_device *pdev) {
    AW9523_LOG("aw9523 remove\n");
    //cancel_delayed_work_sync(&aw9523_key->work);
    //cancel_work_sync(&aw9523_key->eint_work);
    i2c_del_driver(&aw9523_i2c_driver);
    return 0;
}

static int aw9523_key_probe(struct platform_device *pdev) {
    int ret;

    AW9523_LOG("%s start!\n", __func__);

    ret = aw9523_pinctrl_init(pdev);
    if (ret != 0) {
        pr_err("[%s] failed to init aw9523 pinctrl.\n", __func__);
        return ret;
    } else {
        AW9523_LOG("[%s] Success to init aw9523 pinctrl.\n", __func__);
    }

    ret = i2c_add_driver(&aw9523_i2c_driver);
    if (ret != 0) {
        pr_err("[%s] failed to register aw9523 i2c driver.\n", __func__);
        return ret;
    } else {
        AW9523_LOG("[%s] Success to register aw9523 i2c driver.\n", __func__);
    }

    return 0;
}

//#define AW9523_PLATFORM_SUSPEND

#ifdef AW9523_PLATFORM_SUSPEND
static int aw9523_key_suspend(struct platform_device *pdev, pm_message_t state)
{
    AW9523_LOG("%s enter!\n", __func__);
    AW9523_LOG("IRQ disable 3 nosync\n");
    disable_irq_nosync(aw9523_key->irq);
    aw9523_key->irq_enabled = false;

    pinctrl_select_state(aw9523_pin, shdn_low);
    msleep(5);
    AW9523_LOG("%s enter111!\n", __func__);
    cancel_delayed_work_sync(&aw9523_key->work);
    AW9523_LOG("%s enter222!\n", __func__);
    cancel_work_sync(&aw9523_key->eint_work);

    return 0;
}

static int aw9523_key_resume(struct platform_device *pdev)
{
    AW9523_LOG("IRQ Re-enable 3 resume\n");
    AW9523_LOG("%s irq enabled: %d\n", __func__, aw9523_key->irq_enabled);
    if (!aw9523_key->irq_enabled) {
        enable_irq(aw9523_key->irq);
        aw9523_key->irq_enabled = true;
    }

    aw9523_hw_reset();
    aw9523_init_keycfg();
    INIT_DELAYED_WORK(&aw9523_key->work, aw9523_int_work);
    INIT_WORK(&aw9523_key->eint_work, aw9523_key_eint_work);
    return 0;
}
#endif


#ifdef CONFIG_OF
static const struct of_device_id aw9523plt_of_match[] = {
    {.compatible = "mediatek,aw9523_key"},
    {},
};
#endif

static struct platform_driver aw9523_key_driver = {
    .probe = aw9523_key_probe,
    .remove = aw9523_key_remove,
#ifdef AW9523_PLATFORM_SUSPEND
    .suspend = aw9523_key_suspend,
    .resume = aw9523_key_resume,
#endif
    .driver = {
        .name = "aw9523_key",
#ifdef CONFIG_OF
        .of_match_table = aw9523plt_of_match,
#endif
    }
};

static int __init aw9523_key_init(void) {
    int ret;
    AW9523_LOG("%s start\n", __func__);

    ret = platform_driver_register(&aw9523_key_driver);
    if (ret) {
        pr_err("****[%s] Unable to register driver (%d)\n", __func__, ret);
        return ret;
    }
    return 0;
}

static void __exit aw9523_key_exit(void) {
    AW9523_LOG("%s exit\n", __func__);
    platform_driver_unregister(&aw9523_key_driver);
}

module_init(aw9523_key_init);
module_exit(aw9523_key_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC aw9523 Key Driver");
MODULE_LICENSE("GPL");
