//------------------------------------------------------------------------------------------------------------
//
//
//
// a little test ...
// ??? perhaps a separate file for each board and just include it here.... 
//
//------------------------------------------------------------------------------------------------------------
namespace CDC {

const struct CdcResourceDescMap test = {

    .name  = "A little test board name",
    
    .map = {

        {
            .name   = "GPIO-Channel-0",
            .type   = CDC_IT_GPIO,
            
            .gpio = {

                .pin        = ,
                .pinAMode   = CDC_DIO_IN,
                .pinBMode   = CDC_DIO_IN,
            }
        },

        {   
            .name   = "ADC-Channel-0",
            .type   = CDC_IT_ADC,

            .adc =  {   
            
                .adcPin                 = 0, 
                .adcDigitRange          = 1024,
                .adcRefVoltageMilliVolt = 3300
            }   
        },

        {   
            
            .name   = "ADC-Channel-1",
            .type   = CDC_IT_ADC,
            

            .adc =  {   
            
                .adcPin                 = 0, 
                .adcDigitRange          = 1024,
                .adcRefVoltageMilliVolt = 3300
            }   
        }

    }
};

}; // namespace
