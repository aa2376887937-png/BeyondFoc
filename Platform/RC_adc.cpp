#include "RC_adc.h"

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    adc::Adc::All_Adc_ConvCplt_Process(hadc);
}

extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    adc::Adc::All_Adc_InjectedConvCplt_Process(hadc);
}

extern "C" void All_Adc_ConvCplt_Process(ADC_HandleTypeDef *hadc_)
{
    adc::Adc::All_Adc_ConvCplt_Process(hadc_);
}

extern "C" void All_Adc_InjectedConvCplt_Process(ADC_HandleTypeDef *hadc_)
{
    adc::Adc::All_Adc_InjectedConvCplt_Process(hadc_);
}

namespace adc
{
    uint8_t Adc::adc_num = 0;
    Adc *Adc::adc_list[MAX_ADC_NUM] = {nullptr};

    Adc::Adc(ADC_HandleTypeDef &hadc_) : hadc(&hadc_)
    {
        taskENTER_CRITICAL();

        adc_num++;
        if (adc_num > MAX_ADC_NUM) Error_Handler();

        adc_list[adc_num - 1] = this;

        taskEXIT_CRITICAL();
    }

    void Adc::Add_AdcHandle(AdcHandler *hd)
    {
        if (!handlers.Attach(hd)) Error_Handler();
    }

    void Adc::Add_AdcInjectedHandle(AdcInjectedHandler *hd)
    {
        if (!injected_handlers.Attach(hd)) Error_Handler();
    }

    void Adc::All_Adc_ConvCplt_Process(ADC_HandleTypeDef *hadc_)
    {
        for (uint8_t i = 0; i < adc_num; i++)
        {
            if (adc_list[i]->hadc == hadc_)
            {
                adc_list[i]->handlers.Notify([](AdcHandler *handler)
                {
                    handler->Adc_ConvCplt_Process();
                });
                return;
            }
        }
    }

    void Adc::All_Adc_InjectedConvCplt_Process(ADC_HandleTypeDef *hadc_)
    {
        for (uint8_t i = 0; i < adc_num; i++)
        {
            if (adc_list[i]->hadc == hadc_)
            {
                adc_list[i]->injected_handlers.Notify([](AdcInjectedHandler *handler)
                {
                    handler->Adc_InjectedConvCplt_Process();
                });
                return;
            }
        }
    }

    AdcHandler::AdcHandler(Adc *adc_)
    {
        if (adc_ != nullptr)
        {
            adc_->Add_AdcHandle(this);
        }
    }

    AdcInjectedHandler::AdcInjectedHandler(Adc *adc_)
    {
        if (adc_ != nullptr)
        {
            adc_->Add_AdcInjectedHandle(this);
        }
    }
}
