#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H
//BOARD_CUSTOM 
//BOARD_EXG_V3 
#if defined (BOARD_EXG_V3) | defined (BOARD_FULL_EEG_V1)

#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#define LED_1 6
#define LED_2 7

#endif
#endif