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
    pinMode(PIN_A0, OUTPUT);
    analogWriteResolution(8);

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

void setup() {
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
        analogWrite(PIN_A0, pgm_read_byte_far(audioData + sampleIndex));
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
