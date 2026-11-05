#pragma once
#include "main.h"
#include "cmsis_os.h"

#ifdef __cplusplus
#include "Framework/subject.h"
#endif

#ifdef __cplusplus

#define MAX_ADC_NUM 6
#define MAX_ADC_HANDLE_NUM 16

namespace adc
{
    class AdcHandler;
    class AdcInjectedHandler;

    class Adc
    {
    public:
        Adc(ADC_HandleTypeDef &hadc_);
        virtual ~Adc() {}

        inline void Adc_It_Start()
        {
            if (HAL_ADC_Start_IT(hadc) != HAL_OK) Error_Handler();
        }

        inline void Adc_It_Stop()
        {
            if (HAL_ADC_Stop_IT(hadc) != HAL_OK) Error_Handler();
        }

        inline void Adc_Start()
        {
            if (HAL_ADC_Start(hadc) != HAL_OK) Error_Handler();
        }

        inline void Adc_Stop()
        {
            if (HAL_ADC_Stop(hadc) != HAL_OK) Error_Handler();
        }

        inline void Adc_Injected_It_Start()
        {
            if (HAL_ADCEx_InjectedStart_IT(hadc) != HAL_OK) Error_Handler();
        }

        inline void Adc_Injected_It_Stop()
        {
            if (HAL_ADCEx_InjectedStop_IT(hadc) != HAL_OK) Error_Handler();
        }

        inline void Adc_Injected_Start()
        {
            if (HAL_ADCEx_InjectedStart(hadc) != HAL_OK) Error_Handler();
        }

        inline void Adc_Injected_Stop()
        {
            if (HAL_ADCEx_InjectedStop(hadc) != HAL_OK) Error_Handler();
        }

        // ADC 自校准：必须在启动任何转换之前做，能明显减小零偏。
        // 返回 false 表示校准失败（应用层决定是否继续跑）。
        inline bool Adc_Calibrate()
        {
            return HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) == HAL_OK;
        }

        // 读注入组某个 rank 的结果，避免应用层直接碰 HAL 宏
        inline uint32_t Get_Injected(uint32_t rank_) const
        {
            return HAL_ADCEx_InjectedGetValue(hadc, rank_);
        }

        void Add_AdcHandle(AdcHandler *hd);
        void Add_AdcInjectedHandle(AdcInjectedHandler *hd);
        static void All_Adc_ConvCplt_Process(ADC_HandleTypeDef *hadc_);
        static void All_Adc_InjectedConvCplt_Process(ADC_HandleTypeDef *hadc_);

        ADC_HandleTypeDef *hadc;

    private:
        static uint8_t adc_num;
        static Adc *adc_list[MAX_ADC_NUM];

        framework::Subject<AdcHandler, MAX_ADC_HANDLE_NUM> handlers;
        framework::Subject<AdcInjectedHandler, MAX_ADC_HANDLE_NUM> injected_handlers;
    };

    class AdcHandler
    {
    public:
        AdcHandler(Adc *adc_);
        virtual ~AdcHandler() {}

        virtual void Adc_ConvCplt_Process() = 0;
    };

    class AdcInjectedHandler
    {
    public:
        AdcInjectedHandler(Adc *adc_);
        virtual ~AdcInjectedHandler() {}

        virtual void Adc_InjectedConvCplt_Process() = 0;
    };
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

void All_Adc_ConvCplt_Process(ADC_HandleTypeDef *hadc_);
void All_Adc_InjectedConvCplt_Process(ADC_HandleTypeDef *hadc_);

#ifdef __cplusplus
}
#endif
