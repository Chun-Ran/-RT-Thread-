/* main.c — Public/Open-source version
 *
 * Platform: RT-Thread + STM32F103C8T6
 *
 * This file keeps the peripheral initialization and task framework,
 * but removes proprietary signal-processing, calibration, threshold
 * tuning, and business reporting logic.
 *
 * Removed / simplified:
 * - Core adaptive threshold algorithm
 * - Proprietary hit-count/statistical processing
 * - Calibration state machine
 * - Business-specific JSON fields
 * - Sensitive sampling parameters and private constants
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdio.h>
#include <string.h>

#include "i2c_gpio.h"
#include "i2c_ee.h"
#include "stm32f1xx_hal.h"

/* ----------------- Pin / Device Definition ----------------- */
#define LED1_PIN   GET_PIN(B, 1)
#define LED5_PIN   GET_PIN(B, 5)

#define UART2_DEV_NAME  "uart2"
#define PWM3_DEV_NAME   "pwm3"
#define PWM3_CH         2

/* ----------------- Public Demo Constants ----------------- */
#define VREF              3.3f
#define ADC_MAX_VALUE     4095.0f

/*
 * The actual production sampling rate and buffer size have been
 * replaced with demo values.
 */
#define ADC_SAMPLE_RATE_HZ   10000U
#define ADC_BUF_LEN          512U

static __IO uint16_t s_adc_buf[ADC_BUF_LEN];

/* ----------------- HAL Handles ----------------- */
static ADC_HandleTypeDef     s_hadc1;
static DMA_HandleTypeDef     s_hdma_adc1;
static TIM_HandleTypeDef     s_htim2;

/* ----------------- RT-Thread Devices / IPC ----------------- */
static rt_device_t g_uart2 = RT_NULL;
static struct rt_device_pwm *g_pwm3 = RT_NULL;

static rt_mutex_t g_mutex = RT_NULL;
static rt_event_t g_adc_evt;

#define ADC_EVT_HALF   0x01
#define ADC_EVT_FULL   0x02

/* ----------------- Runtime Variables ----------------- */
volatile float g_adc_v = 0.0f;

static float temperature = 0.0f;
static float pressure = 0.0f;
static float altitude = 0.0f;

struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

/* ----------------- Utility Functions ----------------- */

static void ftoa(char *out, int out_sz, float v, int frac)
{
    if (out_sz < 4)
    {
        if (out_sz > 0) out[0] = '\0';
        return;
    }

    int neg = (v < 0.0f);
    if (neg) v = -v;

    int scale = 1;
    for (int i = 0; i < frac; i++) scale *= 10;

    int i_part = (int)v;
    int f_part = (int)((v - (float)i_part) * scale + 0.5f);

    if (f_part >= scale)
    {
        i_part += 1;
        f_part -= scale;
    }

    if (neg)
        rt_snprintf(out, out_sz, "-%d.%0*d", i_part, frac, f_part);
    else
        rt_snprintf(out, out_sz, "%d.%0*d", i_part, frac, f_part);
}

/*
 * Public demo JSON output.
 * Business-specific parameters have been removed.
 */
static void send_demo_json(float voltage, float temp, float press)
{
    char buf[160];
    char s_voltage[24], s_temp[24], s_press[24];

    ftoa(s_voltage, sizeof(s_voltage), voltage, 3);
    ftoa(s_temp,    sizeof(s_temp),    temp,    2);
    ftoa(s_press,   sizeof(s_press),   press,   2);

    rt_snprintf(buf, sizeof(buf),
                "{\"voltage\":%s,\"temperature\":%s,\"pressure\":%s}\r\n",
                s_voltage, s_temp, s_press);

    if (g_uart2)
    {
        rt_device_write(g_uart2, 0, buf, (rt_size_t)rt_strlen(buf));
    }
}

/* ----------------- TIM2 Trigger Configuration ----------------- */

static void TIM2_CC2_Init(uint32_t sample_rate_hz)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    /*
     * Demo timer configuration.
     * Adjust this section according to your board clock tree.
     */
    const uint32_t psc = 71;
    const uint32_t cnt_clk = 72000000UL / (psc + 1UL);

    uint32_t arr = (cnt_clk + sample_rate_hz - 1U) / sample_rate_hz;
    if (arr < 2U) arr = 2U;
    arr -= 1U;

    s_htim2.Instance = TIM2;
    s_htim2.Init.Prescaler         = psc;
    s_htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim2.Init.Period            = arr;
    s_htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_Base_Init(&s_htim2);

    TIM_ClockConfigTypeDef clk = {0};
    clk.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&s_htim2, &clk);

    HAL_TIM_OC_Init(&s_htim2);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode      = TIM_OCMODE_TOGGLE;
    oc.Pulse       = 1;
    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode  = TIM_OCFAST_DISABLE;

    HAL_TIM_OC_ConfigChannel(&s_htim2, &oc, TIM_CHANNEL_2);

    HAL_TIM_Base_Start(&s_htim2);
    HAL_TIM_OC_Start(&s_htim2, TIM_CHANNEL_2);
}

/* ----------------- ADC1 + DMA Initialization ----------------- */

static void ADC1_DMA_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);

    s_hadc1.Instance = ADC1;
    s_hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    s_hadc1.Init.ContinuousConvMode    = DISABLE;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_CC2;
    s_hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.NbrOfConversion       = 1;

    HAL_ADC_Init(&s_hadc1);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_5;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;

    HAL_ADC_ConfigChannel(&s_hadc1, &ch);

    s_hdma_adc1.Instance                 = DMA1_Channel1;
    s_hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    s_hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&s_hdma_adc1);
    __HAL_LINKDMA(&s_hadc1, DMA_Handle, s_hdma_adc1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    HAL_ADCEx_Calibration_Start(&s_hadc1);
}

static void ADC1_DMA_Start(void)
{
    HAL_ADC_Start_DMA(&s_hadc1, (uint32_t *)s_adc_buf, ADC_BUF_LEN);
}

static inline void update_latest_voltage(uint32_t offset_last)
{
    uint16_t raw = s_adc_buf[offset_last];
    g_adc_v = ((float)raw / ADC_MAX_VALUE) * VREF;
}

/* ----------------- HAL ADC Callbacks ----------------- */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        update_latest_voltage((ADC_BUF_LEN / 2) - 1);
        rt_event_send(g_adc_evt, ADC_EVT_HALF);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        update_latest_voltage(ADC_BUF_LEN - 1);
        rt_event_send(g_adc_evt, ADC_EVT_FULL);
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_adc1);
}

/* ----------------- Public Demo ADC Processing Task ----------------- */

/*
 * Proprietary signal-processing logic has been removed.
 *
 * This task only calculates a simple average voltage for demonstration.
 * You may replace this section with your own open-source algorithm.
 */
static void adc_process_task(void *parameter)
{
    while (1)
    {
        rt_uint32_t set = 0;

        if (rt_event_recv(g_adc_evt,
                          ADC_EVT_HALF | ADC_EVT_FULL,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER,
                          &set) != RT_EOK)
        {
            continue;
        }

        uint32_t start = 0;
        uint32_t len = 0;

        if (set & ADC_EVT_HALF)
        {
            start = 0;
            len = ADC_BUF_LEN / 2;
        }
        else if (set & ADC_EVT_FULL)
        {
            start = ADC_BUF_LEN / 2;
            len = ADC_BUF_LEN / 2;
        }

        uint32_t sum = 0;

        for (uint32_t n = 0; n < len; n++)
        {
            sum += s_adc_buf[start + n];
        }

        if (len > 0)
        {
            float avg_raw = (float)sum / (float)len;
            float avg_v = avg_raw / ADC_MAX_VALUE * VREF;

            if (g_mutex && rt_mutex_take(g_mutex, 10) == RT_EOK)
            {
                g_adc_v = avg_v;
                rt_mutex_release(g_mutex);
            }
        }
    }
}

/* ----------------- Public Demo Report Task ----------------- */

static void report_task(void *parameter)
{
    while (1)
    {
        rt_pin_write(LED5_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(LED5_PIN, PIN_LOW);

        if (HP203B_ReadReg(0x0d) & 0x40)
        {
            (void)HP203B_ReadData(&temperature, &pressure, &altitude);
            HP203B_StartConv();
        }

        float voltage_snapshot = 0.0f;

        if (g_mutex && rt_mutex_take(g_mutex, 10) == RT_EOK)
        {
            voltage_snapshot = g_adc_v;
            rt_mutex_release(g_mutex);
        }

        rt_kprintf("voltage=%.3fV, temperature=%.2f, pressure=%.2f\r\n",
                   voltage_snapshot, temperature, pressure);

        send_demo_json(voltage_snapshot, temperature, pressure);

        rt_thread_mdelay(1000);
    }
}

/* ----------------- Main Function ----------------- */

int main(void)
{
    /* PWM demo output */
    g_pwm3 = (struct rt_device_pwm *)rt_device_find(PWM3_DEV_NAME);

    if (g_pwm3)
    {
        /*
         * Demo PWM setting.
         * Replace with project-specific values if needed.
         */
        rt_pwm_set(g_pwm3, PWM3_CH, 100000, 50000);
        rt_pwm_enable(g_pwm3, PWM3_CH);
    }

    /* GPIO output */
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED5_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(LED1_PIN, PIN_LOW);
    rt_pin_write(LED5_PIN, PIN_LOW);

    /* UART2 */
    g_uart2 = rt_device_find(UART2_DEV_NAME);

    if (g_uart2)
    {
        config.baud_rate = 9600;
        rt_device_control(g_uart2, RT_DEVICE_CTRL_CONFIG, &config);
        rt_device_open(g_uart2, RT_DEVICE_OFLAG_WRONLY);
    }

    /* Software I2C and HP203B demo initialization */
    i2c_CfgGpio();
    HP203B_Reset();
    rt_thread_mdelay(100);
    HP203B_StartConv();

    /* IPC */
    g_mutex = rt_mutex_create("imtx", RT_IPC_FLAG_PRIO);
    if (!g_mutex)
    {
        rt_kprintf("mutex create failed\r\n");
        return -1;
    }

    g_adc_evt = rt_event_create("adcev", RT_IPC_FLAG_PRIO);
    if (!g_adc_evt)
    {
        rt_kprintf("event create failed\r\n");
        return -1;
    }

    /* Start ADC + DMA sampling */
    TIM2_CC2_Init(ADC_SAMPLE_RATE_HZ);
    ADC1_DMA_Init();
    ADC1_DMA_Start();

    /* Create tasks */
    rt_thread_t t_adc = rt_thread_create("adc_demo",
                                         adc_process_task,
                                         RT_NULL,
                                         768,
                                         14,
                                         10);

    rt_thread_t t_report = rt_thread_create("report",
                                            report_task,
                                            RT_NULL,
                                            1024,
                                            13,
                                            10);

    if (t_adc)
        rt_thread_startup(t_adc);
    else
        rt_kprintf("adc task create failed\r\n");

    if (t_report)
        rt_thread_startup(t_report);
    else
        rt_kprintf("report task create failed\r\n");

    while (1)
    {
        rt_thread_mdelay(1000);
    }
}