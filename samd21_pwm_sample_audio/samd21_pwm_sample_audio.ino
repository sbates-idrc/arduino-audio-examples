// PWM based on code from MartinL in
// https://forum.arduino.cc/index.php?topic=346731.0
// Sample playback based on 
// https://github.com/arduino-libraries/AudioZero/blob/master/src/AudioZero.cpp

#include "forward_u8_22050.h"

byte* audioData;
uint32_t audioDataLen;
uint32_t sampleIndex;

void TC5_Handler (void) __attribute__ ((weak, alias("Play_Sample_Handler")));

bool tcIsSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void resetTc()
{
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (tcIsSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);
}

void disableTc()
{
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (tcIsSyncing());
}

void configureTc(uint32_t sampleRate)
{
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
    while (GCLK->STATUS.bit.SYNCBUSY);

    resetTc();

    // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

    // Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

    TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
    while (tcIsSyncing());
    
    // Configure interrupt request
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

    // Enable the TC5 interrupt request
    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (tcIsSyncing());
}

void playSample(byte* data, uint32_t length, uint32_t sampleRate)
{
    audioData = data;
    audioDataLen = length;
    sampleIndex = 0;

    configureTc(sampleRate);
}

void stopPlaying()
{
    resetTc();
    disableTc();
}

void sayForward()
{
    playSample(forward_u8_22050_raw, forward_u8_22050_raw_len, 22050);
    delay(3000);
    stopPlaying();
}

void setup()
{
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                      GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                       GCLK_GENCTRL_GENEN |         // Enable GCLK4
                       GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                       GCLK_GENCTRL_ID(4);          // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Enable the port multiplexer for the digital pin D7
    PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
 
    // Connect the TCC0 timer to digital output D7 - port pins are paired odd PMUO and even PMUXE
    // F & E specify the timers: TCC0, TCC1 and TCC2
    PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

    // Feed GCLK4 to TCC0 and TCC1
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                       GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                       GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

    // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
    REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |           // Reverse the output polarity on all TCC0 outputs
                     TCC_WAVE_WAVEGEN_DSBOTH;      // Setup dual slope PWM on TCC0
    while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

    // Each timer counts up to a maximum or TOP value set by the PER register,
    // this determines the frequency of the PWM operation:
    REG_TCC0_PER = 255;
    while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
 
    REG_TCC0_CC3 = 0;         // TCC0 CC3 - on D7
    while (TCC0->SYNCBUSY.bit.CC3);                // Wait for synchronization
 
    // Divide the 48MHz signal by 1 giving 48MHz TCC0 timer tick and enable the outputs
    REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |   // Divide GCLK4 by 1
                      TCC_CTRLA_ENABLE;            // Enable the TCC0 output
    while (TCC0->SYNCBUSY.bit.ENABLE);             // Wait for synchronization
}

void loop() {
    sayForward();
    delay(4000);
}

#ifdef __cplusplus
extern "C" {
#endif

void Play_Sample_Handler (void)
{
    if (sampleIndex < audioDataLen - 1) {
        // TCC0 CC3 - on D7
        REG_TCC0_CC3 = pgm_read_byte_far(audioData + sampleIndex);
        ++sampleIndex;

        // Clear the interrupt
        TC5->COUNT16.INTFLAG.bit.MC0 = 1;
    } else {
        // This will loop until stopPlaying() is called
        sampleIndex = 0;
        TC5->COUNT16.INTFLAG.bit.MC0 = 1;
    }
}

#ifdef __cplusplus
}
#endif
