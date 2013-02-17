/*
** =========================================================================
** File:
**     tspdrv.c
**
** Description:
**     TouchSense Kernel Module main entry-point.
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
** =========================================================================
*/

#include <linux/module.h>	
#include <linux/kernel.h>	
#include <linux/init.h>		
#include <linux/slab.h>		
#include <linux/mutex.h>	
#include <linux/delay.h>	
#include <linux/i2c.h>		
#include <linux/pm.h>		
#include <linux/miscdevice.h>	
#include <linux/mod_devicetable.h> 
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/kobject.h>	
#include <linux/gpio.h>		
#include <linux/input.h>	
#include <linux/string.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>	
#include <mach/isa1200.h>
#include <mach/board-sec-ux500.h>
#include <linux/mfd/dbx500-prcmu.h>
#include "../../staging/android/timed_output.h"
#include "tspdrv.h"

#define SCTRL           (0)     /* 0x0F, System(LDO) Register Group 0*/
#define HCTRL0          (0x30)     /* 0x09 */ /* Haptic Motor Driver Control Register Group 0*/
#define HCTRL1          (0x31)     /* 0x4B */ /* Haptic Motor Driver Control Register Group 1*/
#define HCTRL2          (0x32)     /* 0x00*/ /* Haptic Motor Driver Control Register Group 2*/
#define HCTRL3          (0x33)     /* 0x13 */ /* Haptic Motor Driver Control Register Group 3*/
#define HCTRL4          (0x34)     /* 0x00 */ /* Haptic Motor Driver Control Register Group 4*/
#define HCTRL5          (0x35)     /* 0x6B */ /* Haptic Motor Driver Control Register Group 5*/
#define HCTRL6          (0x36)     /* 0xD6 */ /* Haptic Motor Driver Control Register Group 6*/

#define LDO_VOLTAGE_27V 0x0C
#define LDO_VOLTAGE_30V 0x0F

#ifdef CONFIG_MACH_JANICE
#define PWM_PLLDIV_DEFAULT		0x02
#define PWM_FREQ_DEFAULT		0x00
#define PWM_PERIOD_DEFAULT		0x77
#define PWM_DUTY_DEFAULT		0x3B
#define LDO_VOLTAGE_DEFAULT		LDO_VOLTAGE_30V
#elif defined(CONFIG_MACH_GAVINI)
#define PWM_PLLDIV_DEFAULT		0x02
#define PWM_FREQ_DEFAULT		0x00
#define PWM_PERIOD_DEFAULT		0x8C
#define PWM_DUTY_DEFAULT		0x46
#define LDO_VOLTAGE_DEFAULT		LDO_VOLTAGE_27V
#endif

#define NUM_ACTUATORS        1
#define MAX_TIMEOUT         10000 /* 10s */

struct vibrator {
	struct wake_lock wklock;
	struct hrtimer timer;
	struct mutex lock;
	struct work_struct work;
	bool running;
};

static u_int32_t g_nPWM_PLLDiv = PWM_PLLDIV_DEFAULT;
static u_int32_t g_nPWM_Freq = PWM_FREQ_DEFAULT;
static u_int32_t g_nPWM_Period = PWM_PERIOD_DEFAULT;
static u_int32_t g_nPWM_Duty = PWM_DUTY_DEFAULT;
static u_int32_t g_nLDO_Voltage = LDO_VOLTAGE_DEFAULT;
static struct miscdevice miscdev; 
static bool g_bAmpEnabled;
struct isa1200_data *isa_data;
static struct vibrator vibdata;

int immvibe_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	int	ret;
	u8	data[2];

	data[0] = reg;
	data[1] = val;
	ret = i2c_master_send(client, data, 2);
	if (ret < 0) {
		pr_err("Failed to send data to isa1200 [errno=%d]", ret);
		return ret;
	} 
	return ret;
}

/*
*  Called to disable amp (disable output force)
*/
static int32_t ImmVibeSPI_ForceOut_AmpDisable(u_int8_t nActuatorIndex)
{
	if (g_bAmpEnabled == true) {
		g_bAmpEnabled = false;
		immvibe_i2c_write(isa_data->client, HCTRL0, 0x00);

		gpio_direction_output(isa_data->pdata->mot_hen_gpio, 0);
		gpio_direction_output(isa_data->pdata->mot_len_gpio, 0);

		clk_disable(isa_data->mot_clk);
	}

	return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/

static int32_t ImmVibeSPI_ForceOut_AmpEnable(u_int8_t nActuatorIndex)
{
	if (g_bAmpEnabled == false) {
		clk_enable(isa_data->mot_clk);

		gpio_direction_output(isa_data->pdata->mot_hen_gpio, 1);
		gpio_direction_output(isa_data->pdata->mot_len_gpio, 1);

		udelay(200);

		immvibe_i2c_write(isa_data->client, SCTRL, g_nLDO_Voltage);

		/* If the PWM frequency is 44.8kHz, then the output frequency will be 44.8/div_factor
		HCTRL0[1:0] is the div_factor, below setting sets div_factor to 256, so o/p frequency is 175 Hz
		*/
		immvibe_i2c_write(isa_data->client, HCTRL0, 0x11);
		immvibe_i2c_write(isa_data->client, HCTRL1, 0xC0);
		immvibe_i2c_write(isa_data->client, HCTRL2, 0x00);
		immvibe_i2c_write(isa_data->client, HCTRL3, (0x03 + (g_nPWM_PLLDiv<<4)));
		immvibe_i2c_write(isa_data->client, HCTRL4, g_nPWM_Freq);
		immvibe_i2c_write(isa_data->client, HCTRL5, g_nPWM_Duty);
		immvibe_i2c_write(isa_data->client, HCTRL6, g_nPWM_Period);

		/* Haptic Enable + PWM generation mode */
		immvibe_i2c_write(isa_data->client, HCTRL0, 0x91);

		g_bAmpEnabled = true;	/* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
	}
	return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
static int32_t ImmVibeSPI_ForceOut_Terminate(void)
{
	gpio_direction_output(isa_data->pdata->mot_hen_gpio, 0);
	gpio_direction_output(isa_data->pdata->mot_len_gpio, 0);

	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle
*/
static int32_t ImmVibeSPI_ForceOut_SetSamples(u_int8_t nActuatorIndex, u_int16_t nOutputSignalBitDepth, u_int16_t nBufferSizeInBytes, int8_t * pForceOutputBuffer)
{
    unsigned int duty;
    int8_t nForce;

	switch (nOutputSignalBitDepth) {
	case 8:
		/* pForceOutputBuffer is expected to contain 1 byte */
		if (nBufferSizeInBytes != 1)
			return VIBE_E_FAIL;

		nForce = pForceOutputBuffer[0];
		break;
	case 16:
		/* pForceOutputBuffer is expected to contain 2 byte */
		if (nBufferSizeInBytes != 2)
			return VIBE_E_FAIL;

		/* Map 16-bit value to 8-bit */
		nForce = ((int16_t *)pForceOutputBuffer)[0] >> 8;
		break;
	default:
		/* Unexpected bit depth */
		return VIBE_E_FAIL;
    }

    if (nForce == 0) {
		immvibe_i2c_write(isa_data->client, HCTRL5, g_nPWM_Duty);
    } else {
		duty = g_nPWM_Duty + ((g_nPWM_Duty-1)*nForce)/127;
		immvibe_i2c_write(isa_data->client, HCTRL5, duty);
    }
    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
static int32_t ImmVibeSPI_Device_GetName(u_int8_t nActuatorIndex, char *szDevName, int nSize)
{
    if ((!szDevName) || (nSize < 1))
		return VIBE_E_FAIL;

    pr_debug("ImmVibeSPI_Device_GetName.\n");

    strncpy(szDevName, "Generic Linux Device", nSize-1);
    szDevName[nSize - 1] = '\0';    /* make sure the string is NULL terminated */

    return VIBE_S_SUCCESS;
}


/* Device name and version information */
#define VERSION_STR " v3.4.55.5\n"                  /* DO NOT CHANGE - this is auto-generated */
#define VERSION_STR_LEN 16                          /* account extra space for future extra digits in version number */
static char g_szDeviceName[(VIBE_MAX_DEVICE_NAME_LENGTH
				+ VERSION_STR_LEN) * NUM_ACTUATORS];       /* initialized in init_module */
static size_t g_cchDeviceName;                      /* initialized in init_module */

/* Flag indicating whether the driver is in use */
static char g_bIsPlaying;

/* Buffer to store data sent to SPI */
#define SPI_BUFFER_SIZE (NUM_ACTUATORS * (VIBE_OUTPUT_SAMPLE_SIZE + SPI_HEADER_SIZE))
static int g_bStopRequested;
static actuator_samples_buffer g_SamplesBuffer[NUM_ACTUATORS] = {{0},};
static char g_cWriteBuffer[SPI_BUFFER_SIZE];


#define WATCHDOG_TIMEOUT    10  /* 10 timer cycles = 50ms */ 

/* Global variables */
static bool g_bTimerStarted = false;
static struct hrtimer g_tspTimer;
static ktime_t g_ktFiveMs;
static int g_nWatchdogCounter = 0;
DEFINE_SEMAPHORE(g_hMutex);


static inline int VibeSemIsLocked(struct semaphore *lock)
{
#if ((LINUX_VERSION_CODE & 0xFFFFFF) < KERNEL_VERSION(2,6,27))
    return atomic_read(&lock->count) != 1;
#else
    return (lock->count) != 1;
#endif
}

static enum hrtimer_restart tsp_timer_interrupt(struct hrtimer *timer)
{
    /* Scheduling next timeout value right away */
    hrtimer_forward_now(timer, g_ktFiveMs);

    if(g_bTimerStarted)
    {
        if (VibeSemIsLocked(&g_hMutex)) up(&g_hMutex);
    }

    return HRTIMER_RESTART;
}

static void VibeOSKernelLinuxStopTimer(void)
{
    int i;

    if (g_bTimerStarted)
    {
        g_bTimerStarted = false;
        hrtimer_cancel(&g_tspTimer);
    }

    /* Reset samples buffers */
    for (i = 0; i < NUM_ACTUATORS; i++)
    {
        g_SamplesBuffer[i].nIndexPlayingBuffer = -1;
        g_SamplesBuffer[i].actuatorSamples[0].nBufferSize = 0;
        g_SamplesBuffer[i].actuatorSamples[1].nBufferSize = 0;
    }
    g_bStopRequested = false;
    g_bIsPlaying = false;
} 

static int VibeOSKernelProcessData(void* data)
{
    int i;
    int nActuatorNotPlaying = 0;

    for (i = 0; i < NUM_ACTUATORS; i++) 
    {
        actuator_samples_buffer *pCurrentActuatorSample = &(g_SamplesBuffer[i]);

        if (-1 == pCurrentActuatorSample->nIndexPlayingBuffer)
        {
            nActuatorNotPlaying++;
            if ((NUM_ACTUATORS == nActuatorNotPlaying) && ((++g_nWatchdogCounter) > WATCHDOG_TIMEOUT))
            {
                int8_t cZero[1] = {0};

                /* Nothing to play for all actuators, turn off the timer when we reach the watchdog tick count limit */
                ImmVibeSPI_ForceOut_SetSamples(i, 8, 1, cZero);
                ImmVibeSPI_ForceOut_AmpDisable(i);
                VibeOSKernelLinuxStopTimer();

                /* Reset watchdog counter */
                g_nWatchdogCounter = 0;
            }
        }
        else
        {
            /* Play the current buffer */
            if (VIBE_E_FAIL == ImmVibeSPI_ForceOut_SetSamples(
                pCurrentActuatorSample->actuatorSamples[(int)pCurrentActuatorSample->nIndexPlayingBuffer].nActuatorIndex, 
                pCurrentActuatorSample->actuatorSamples[(int)pCurrentActuatorSample->nIndexPlayingBuffer].nBitDepth, 
                pCurrentActuatorSample->actuatorSamples[(int)pCurrentActuatorSample->nIndexPlayingBuffer].nBufferSize,
                pCurrentActuatorSample->actuatorSamples[(int)pCurrentActuatorSample->nIndexPlayingBuffer].dataBuffer))
            {
                /* VIBE_E_FAIL means NAK has been handled. Schedule timer to restart 5 ms from now */
                hrtimer_forward_now(&g_tspTimer, g_ktFiveMs);
            }

            pCurrentActuatorSample->nIndexOutputValue += pCurrentActuatorSample->actuatorSamples[(int)pCurrentActuatorSample->nIndexPlayingBuffer].nBufferSize;

            if (pCurrentActuatorSample->nIndexOutputValue >= pCurrentActuatorSample->actuatorSamples[(int)pCurrentActuatorSample->nIndexPlayingBuffer].nBufferSize)
            {
                /* Reach the end of the current buffer */
                pCurrentActuatorSample->actuatorSamples[(int)pCurrentActuatorSample->nIndexPlayingBuffer].nBufferSize = 0;

                /* Switch buffer */
                (pCurrentActuatorSample->nIndexPlayingBuffer) ^= 1;
                pCurrentActuatorSample->nIndexOutputValue = 0;

                /* Finished playing, disable amp for actuator (i) */
                if (g_bStopRequested)
                {
                    pCurrentActuatorSample->nIndexPlayingBuffer = -1; 

                    ImmVibeSPI_ForceOut_AmpDisable(i);
                }
            }
        }
    }

    /* If finished playing, stop timer */
    if (g_bStopRequested)
    {
        VibeOSKernelLinuxStopTimer();

        /* Reset watchdog counter */
        g_nWatchdogCounter = 0;

        if (VibeSemIsLocked(&g_hMutex)) up(&g_hMutex);
        return 1;   /* tell the caller this is the last iteration */
    }

    return 0;
}

static void VibeOSKernelLinuxInitTimer(void)
{
    /* Get a 5,000,000ns = 5ms time value */
    g_ktFiveMs = ktime_set(0, 5000000);

    hrtimer_init(&g_tspTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

    /* Initialize a 5ms-timer with tsp_timer_interrupt as timer callback (interrupt driven)*/
    g_tspTimer.function = tsp_timer_interrupt;
}

static void VibeOSKernelLinuxStartTimer(void)
{
    int i;
    int res;

    /* Reset watchdog counter */
    g_nWatchdogCounter = 0;

    if (!g_bTimerStarted)
    {
        if (!VibeSemIsLocked(&g_hMutex)) res = down_interruptible(&g_hMutex); /* start locked */

        g_bTimerStarted = true;

        /* Start the timer */
        hrtimer_start(&g_tspTimer, g_ktFiveMs, HRTIMER_MODE_REL);

        /* Don't block the write() function after the first sample to allow the host sending the next samples with no delay */
        for (i = 0; i < NUM_ACTUATORS; i++)
        {
            if ((g_SamplesBuffer[i].actuatorSamples[0].nBufferSize) || (g_SamplesBuffer[i].actuatorSamples[1].nBufferSize))
            {
                g_SamplesBuffer[i].nIndexOutputValue = 0;
                return;
            }
        }
    }

    if (0 != VibeOSKernelProcessData(NULL)) return;

    res = down_interruptible(&g_hMutex);  /* wait for the mutex to be freed by the timer */
    if (res != 0)
    {
        pr_info("VibeOSKernelLinuxStartTimer: down_interruptible interrupted by a signal.\n");
    }
  
}

static void VibeOSKernelLinuxTerminateTimer(void)
{
    VibeOSKernelLinuxStopTimer();
    if (VibeSemIsLocked(&g_hMutex)) up(&g_hMutex);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	schedule_work(&vibdata.work);
	return HRTIMER_NORESTART;
}

static void vibrator_work(struct work_struct *work)
{
	if (!vibdata.running)
		return;

	vibdata.running = false;

	immvibe_i2c_write(isa_data->client, HCTRL0, 0x00);
	gpio_direction_output(isa_data->pdata->mot_hen_gpio, 0);
	gpio_direction_output(isa_data->pdata->mot_len_gpio, 0);

	clk_disable(isa_data->mot_clk);
	wake_unlock(&vibdata.wklock);
}

static void janice_vibrator_enable(struct timed_output_dev *dev, int value)
{
	mutex_lock(&vibdata.lock);

	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&vibdata.timer);
	cancel_work_sync(&vibdata.work);

	if (value) {
		wake_lock(&vibdata.wklock);
		if (!vibdata.running) {
			clk_enable(isa_data->mot_clk);
			gpio_direction_output(isa_data->pdata->mot_hen_gpio, 1);
			gpio_direction_output(isa_data->pdata->mot_len_gpio, 1);

			udelay(200);

			immvibe_i2c_write(isa_data->client, SCTRL, g_nLDO_Voltage);

			/* If the PWM frequency is 44.8kHz, then the output frequency will be 44.8/div_factor
			HCTRL0[1:0] is the div_factor, below setting sets div_factor to 256, so o/p frequency is 175 Hz
			*/
			immvibe_i2c_write(isa_data->client, HCTRL0, 0x11);
			immvibe_i2c_write(isa_data->client, HCTRL1, 0xC0);
			immvibe_i2c_write(isa_data->client, HCTRL2, 0x00);
			immvibe_i2c_write(isa_data->client, HCTRL3, (0x03 + (g_nPWM_PLLDiv<<4)));
			immvibe_i2c_write(isa_data->client, HCTRL4, g_nPWM_Freq);
			immvibe_i2c_write(isa_data->client, HCTRL5, g_nPWM_Duty);
			immvibe_i2c_write(isa_data->client, HCTRL6, g_nPWM_Period);

			/* PWM generation mode */
			immvibe_i2c_write(isa_data->client, HCTRL0, 0x91);
			/* Duty 0x64 == nForce 90 */
			immvibe_i2c_write(isa_data->client, HCTRL5, 0x64);

			vibdata.running = true;
		} else
			pr_info("%s: value = %d, already running, rescheduling timer\n",
				__func__, value);

		if (value > 0) {
			value = value + 30;
			if (value > MAX_TIMEOUT)
				value = MAX_TIMEOUT;

			hrtimer_start(&vibdata.timer,
				ns_to_ktime((u64)value * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
		}
	}

	mutex_unlock(&vibdata.lock);
}


static int janice_vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibdata.timer)) {
		ktime_t r = hrtimer_get_remaining(&vibdata.timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static int open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	return 0;
}

static int release(struct inode *inode, struct file *file)
{
    VibeOSKernelLinuxStopTimer();
    file->private_data = (void *)NULL;
    module_put(THIS_MODULE);
    return 0;
}

static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	const size_t nBufSize = (g_cchDeviceName > (size_t)(*ppos)) ? min(count, g_cchDeviceName - (size_t)(*ppos)) : 0;

	/* End of buffer, exit */
	if (0 == nBufSize)
		return 0;

	if (0 != copy_to_user(buf, g_szDeviceName + (*ppos), nBufSize)) {
		/* Failed to copy all the data, exit */
		pr_err("tspdrv: copy_to_user failed.\n");
		return 0;
	}

	/* Update file position and return copied buffer size */
	*ppos += nBufSize;
	return nBufSize;
}

static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
	int i = 0;

	*ppos = 0;  /* file position not used, always set to 0 */

	/*
	** Prevent unauthorized caller to write data.
	** TouchSense service is the only valid caller.
	*/
	if (file->private_data != (void *)TSPDRV_MAGIC_NUMBER) {
		pr_err("tspdrv: unauthorized write.\n");
		return 0;
	}

	/* Copy immediately the input buffer */
	if (0 != copy_from_user(g_cWriteBuffer, buf, count)) {
		/* Failed to copy all the data, exit */
		pr_err("tspdrv: copy_from_user failed.\n");
		return 0;
	}

	/* Check buffer size */
	if ((count <= SPI_HEADER_SIZE) || (count > SPI_BUFFER_SIZE)) {
		pr_err("tspdrv: invalid write buffer size.\n");
		return 0;
	}

	while (i < count) {
		int nIndexFreeBuffer;   /* initialized below */

		samples_buffer* pInputBuffer = (samples_buffer *)(&g_cWriteBuffer[i]);

		if ((i + SPI_HEADER_SIZE) >= count) {
			/*
			** Index is about to go beyond the buffer size.
			** (Should never happen).
			*/
			pr_err("tspdrv: invalid buffer index.\n");
		}

		/* Check bit depth */
		if (8 != pInputBuffer->nBitDepth) {
			pr_err("tspdrv: invalid bit depth. Use default value (8).\n");
		}

		/* The above code not valid if SPI header size is not 3 */
#if (SPI_HEADER_SIZE != 3)
#error "SPI_HEADER_SIZE expected to be 3"
#endif

		/* Check buffer size */
		if ((i + SPI_HEADER_SIZE + pInputBuffer->nBufferSize) > count) {
			/*
			** Index is about to go beyond the buffer size.
			** (Should never happen).
			*/
			pr_err("tspdrv: invalid data size.\n");
		}

		/* Check actuator index */
		if (NUM_ACTUATORS <= pInputBuffer->nActuatorIndex) {
			pr_err("tspdrv: invalid actuator index.\n");
			i += (SPI_HEADER_SIZE + pInputBuffer->nBufferSize);
			continue;
		}

		if (0 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[0].nBufferSize)
			nIndexFreeBuffer = 0;
		else if (0 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[1].nBufferSize)
			 nIndexFreeBuffer = 1;
		else {
			/* No room to store new samples  */
			pr_err("tspdrv: no room to store new samples.\n");
			return 0;
		}

		/* Store the data in the free buffer of the given actuator */
		memcpy(&(g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[nIndexFreeBuffer]),
					&g_cWriteBuffer[i], (SPI_HEADER_SIZE + pInputBuffer->nBufferSize));

		/* If the no buffer is playing, prepare to play g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[nIndexFreeBuffer] */
		if (-1 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexPlayingBuffer) {
		   g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexPlayingBuffer = nIndexFreeBuffer;
		   g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexOutputValue = 0;
		}

		/* Increment buffer index */
		i += (SPI_HEADER_SIZE + pInputBuffer->nBufferSize);
	}

	/* Start the work after receiving new output force */
	g_bIsPlaying = true;
	VibeOSKernelLinuxStartTimer();

	return count;
}

static long ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case TSPDRV_STOP_KERNEL_TIMER:
		/*
		** As we send one sample ahead of time, we need to finish playing the last sample
		** before stopping the timer. So we just set a flag here.
		*/
		if (true == g_bIsPlaying)
			g_bStopRequested = true;

		VibeOSKernelProcessData(NULL);


		break;

	case TSPDRV_MAGIC_NUMBER:
		file->private_data = (void *)TSPDRV_MAGIC_NUMBER;
		break;

	case TSPDRV_ENABLE_AMP:
		ImmVibeSPI_ForceOut_AmpEnable(arg);
		break;

	case TSPDRV_DISABLE_AMP:
		/* Small fix for now to handle proper combination of TSPDRV_STOP_KERNEL_TIMER and TSPDRV_DISABLE_AMP together */
		/* If a stop was requested, ignore the request as the amp will be disabled by the timer proc when it's ready */
		if (!g_bStopRequested) {
			ImmVibeSPI_ForceOut_AmpDisable(arg);
		}
		break;

	case TSPDRV_GET_NUM_ACTUATORS:
		return NUM_ACTUATORS;
	}

	return 0;
}

static int suspend(struct platform_device *pdev, pm_message_t state)
{
	if (g_bIsPlaying) {
		// printk((KERN_INFO "tspdrv: can't suspend, still playing effects.\n"));
		return -EBUSY;
	} else {
		return 0;
	}
}

static int resume(struct platform_device *pdev)
{
	return 0;   /* can resume */
}


/* -------------------------------------------------------------------------
 * I2C interface functions
 * ------------------------------------------------------------------------- */

int immvibe_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	g_bAmpEnabled = false;
	gpio_direction_output(isa_data->pdata->mot_hen_gpio, 0);
	gpio_direction_output(isa_data->pdata->mot_len_gpio, 0);
	return 0;
}

int immvibe_i2c_resume(struct i2c_client *client)
{
	int ret = 0;
	return ret;
}

static struct platform_driver platdrv = {
    .suspend =  suspend,
    .resume =   resume,
    .driver = {
		.name = MODULE_NAME,
    },
};

static int __devinit immvibe_i2c_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct isa1200_platform_data *pdata;
	int i;
	int ret = 0;
	g_cchDeviceName = 0;

	pr_debug("%s(), client = %s", __func__, client->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->adapter->dev, "%s failed %d\n", __func__, ret);
		return -ENODEV;
	}

	pdata = (struct isa1200_platform_data *) client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "platform data required for isa1200\n");
		return -EINVAL;
	}
	pdata = client->dev.platform_data;

	isa_data = kzalloc(sizeof(struct isa1200_data), GFP_KERNEL);
	if (isa_data == NULL) {
		dev_err(&client->dev, "error allocating memory\n");
		ret = -ENOMEM;
		goto out_alloc_data_failed;
	}

	isa_data->addr = client->addr;
	isa_data->client = client;
	isa_data->pdata = pdata;

#ifdef CONFIG_MACH_JANICE
	if (system_rev >= JANICE_R0_3)
		isa_data->mot_clk  = clk_get_sys("mot-pwm0", NULL);
	else
		isa_data->mot_clk  = clk_get_sys("mot-pwm1", NULL);
#elif defined(CONFIG_MACH_GAVINI)
	if (system_rev > GAVINI_R0_0_B)
		isa_data->mot_clk  = clk_get_sys("mot-pwm0", NULL);
	else
		isa_data->mot_clk  = clk_get_sys("mot-pwm1", NULL);
#endif

	if (pdata->hw_setup) {
		ret = pdata->hw_setup();
		if (ret < 0) {
			pr_err("Failed to setup GPIOs for Vibrator [errno=%d]", ret);
			goto out_gpio_failed;
		}
	}

	i2c_set_clientdata(client, isa_data);

	isa_data->input = input_allocate_device();
	if (!isa_data->input) {
		pr_err("Failed to allocate Vibrator input device.");
		return -ENOMEM;
	}

	isa_data->input->name = "Vibrator";

	ret = input_register_device(isa_data->input);
	if (ret < 0) {
		pr_err("Failed to register Vibrator input device [errno=%d]", ret);
		input_free_device(isa_data->input);
		return ret;
	}

	/* initialize tspdrv module */
	ret = misc_register(&miscdev);
	if(ret) {	
		pr_err("[tspdrv] Failed to register miscdevice!\n");
	 	input_free_device(isa_data->input);
		return ret;
	}

	ret = platform_driver_register(&platdrv);
	if(ret) {
		pr_err("[tspdrv] Failed to register platform driver\n");
		input_free_device(isa_data->input);
		return ret;
	}
        VibeOSKernelLinuxInitTimer();

	for (i = 0; i < NUM_ACTUATORS; i++) {
		char *szName = g_szDeviceName + g_cchDeviceName;
		ImmVibeSPI_Device_GetName(i, szName, VIBE_MAX_DEVICE_NAME_LENGTH);
		strcat(szName, VERSION_STR);
		g_cchDeviceName += strlen(szName);
		g_SamplesBuffer[i].nIndexPlayingBuffer = -1; 
		g_SamplesBuffer[i].actuatorSamples[0].nBufferSize = 0;
		g_SamplesBuffer[i].actuatorSamples[1].nBufferSize = 0;
	}

	return ret;

out_gpio_failed:
	clk_put(isa_data->mot_clk);
	kfree(isa_data);

out_alloc_data_failed:
	return ret;
}

static int __devexit immvibe_i2c_remove(struct i2c_client* client)
{	
	return 0;
}

static const struct i2c_device_id immvibe_id[] = {
	{"immvibe", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, immvibe_id);

static struct i2c_driver immvibe_i2c_driver = {
	.driver = {
		.name = "immvibe",			/* immersion touchsense player driver */
		.owner = THIS_MODULE,
	},
	.id_table = immvibe_id,
	.probe    = immvibe_i2c_probe,
	.remove   = immvibe_i2c_remove,
	.suspend = immvibe_i2c_suspend,
	.resume = immvibe_i2c_resume,
};

/* Device data */ 

static struct file_operations fops = {
    .owner =    THIS_MODULE,
    .read =     read,
    .write =    write,
    .unlocked_ioctl =    ioctl,
    .open =     open,
    .release =  release,
    .llseek =   default_llseek
};


static struct miscdevice miscdev = {
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     "tspdrv",
	.fops =     &fops
};

static struct timed_output_dev to_dev = {
	.name		= "vibrator",
	.get_time	= janice_vibrator_get_time,
	.enable		= janice_vibrator_enable,
};


static int __init immvibe_init(void)
{
	int ret = 0;
	printk(KERN_ERR "%s\n", __func__);
	ret = i2c_add_driver(&immvibe_i2c_driver);

	if (ret < 0)
		pr_err("%s(): Failed to add i2c driver for ISA1200, err: %d\n", __func__, ret);

	hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibdata.timer.function = vibrator_timer_func;
	INIT_WORK(&vibdata.work, vibrator_work);
	wake_lock_init(&vibdata.wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&vibdata.lock);
	ret = timed_output_dev_register(&to_dev);

	if (ret < 0)
		goto err_to_dev_reg;

	return 0;

err_to_dev_reg:
	pr_err("%s(): Failed to register timed_output vibrator, err: %d\n", __func__, ret);
	mutex_destroy(&vibdata.lock);
	wake_lock_destroy(&vibdata.wklock);

	return ret;
}

static void __exit immvibe_exit(void)
{
	printk(KERN_DEBUG "%s\n", __func__);
	i2c_del_driver(&immvibe_i2c_driver);
	timed_output_dev_unregister(&to_dev);
    	VibeOSKernelLinuxTerminateTimer();
	ImmVibeSPI_ForceOut_Terminate();
	platform_driver_unregister(&platdrv);
	misc_deregister(&miscdev);
}

module_init(immvibe_init);
module_exit(immvibe_exit);

/* Module info */
MODULE_AUTHOR("Immersion Corporation");
MODULE_DESCRIPTION("TouchSense Kernel Module");
MODULE_LICENSE("GPL v2");
