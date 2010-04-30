/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id: wm8985.c 21798 2009-07-12 09:43:44Z kugel $
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#include "i2c.h"
#include "wmcodec.h"
#include "hardware/i2s.h"
#include "hardware/wmcodec.h"
#include "clock.h"
#include "dma.h"
#include "util.h"
#include "openiboot-asmhelpers.h"

const void* pcm_buffer;
uint32_t pcm_buffer_size;

int dma_controller = -1;
int dma_channel = -1;

volatile static int transfersDone;

volatile static int stopTransfers;

void audiohw_preinit();

void audiohw_init()
{
	clock_gate_switch(I2S0_CLOCK, ON);
	clock_gate_switch(I2S1_CLOCK, ON);
	audiohw_preinit();
}

void wmcodec_write(int reg, int data)
{
	uint8_t buffer[3];

    buffer[0] = reg & 0xFF;
    buffer[1] = (data >> 8) & 0xFF;
    buffer[2] = data & 0xFF;

	i2c_tx(WMCODEC_I2C, WMCODEC_I2C_SLAVE_ADDR, buffer, 3);
}

static void iis_transfer_done(int status, int controller, int channel)
{
	++transfersDone;
	CleanCPUDataCache();
	dma_finish(controller, channel, 0);
	dma_controller = -1;
	dma_channel = -1;
	bufferPrintf("audio: playback complete\r\n");
	// replace the above line with this bottom one for repeat
	// dma_perform((uint32_t)pcm_buffer, DMA_WM_I2S_TX, pcm_buffer_size, 0, &controller, &channel);
}

int audiohw_transfers_done()
{
	return transfersDone;
}

void audiohw_pause()
{
	if(dma_controller == -1)
		return;

	dma_pause(dma_controller, dma_channel);
}

void audiohw_resume()
{
	if(dma_controller == -1)
		return;

	dma_resume(dma_controller, dma_channel);
}

uint32_t audiohw_get_position()
{
	if(dma_controller == -1)
		return 0;

	return dma_srcpos(dma_controller, dma_channel);
}

uint32_t audiohw_get_total()
{
	if(dma_controller == -1)
		return 0;

	return pcm_buffer_size;
}

void audiohw_play_pcm(const void* addr_in, uint32_t size, int use_speaker)
{
	pcm_buffer = addr_in;
	pcm_buffer_size = size;

	uint32_t i2sController;
	uint32_t dma;

	if(use_speaker)
	{
		i2sController = BB_I2S;
		dma = DMA_BB_I2S_TX;
	}
	else
	{
		i2sController = WM_I2S;
		dma = DMA_WM_I2S_TX;
	}

	SET_REG(i2sController + I2S_TXCON,
			(1 << 24) |  /* undocumented */
			(1 << 20) |  /* undocumented */
			(0 << 16) |  /* burst length */
			(0 << 15) |  /* 0 = falling edge */
			(0 << 13) |  /* 0 = basic I2S format */
			(0 << 12) |  /* 0 = MSB first */
			(0 << 11) |  /* 0 = left channel for low polarity */
			(3 << 8) |   /* MCLK divider */
			(0 << 5) |   /* 0 = 16-bit */
			(0 << 3) |   /* bit clock per frame */
			(1 << 0));    /* channel index */

	int controller = 0;
	int channel = 0;

	transfersDone = 0;
	stopTransfers = 0;
	
	CleanCPUDataCache();

	dma_request(DMA_MEMORY, 2, 1, dma, 2, 1, &controller, &channel, iis_transfer_done);

	dma_perform((uint32_t)pcm_buffer, dma, pcm_buffer_size, 0, &controller, &channel);

	dma_controller = controller;
	dma_channel = channel;

	SET_REG(i2sController + I2S_CLKCON, (1 << 0)); /* 1 = power on */
	SET_REG(i2sController + I2S_TXCOM, 
			(0 << 3) |   /* 1 = transmit mode on */
			(1 << 2) |   /* 1 = I2S interface enable */
			(1 << 1) |   /* 1 = DMA request enable */
			(0 << 0));    /* 0 = LRCK on */

	CleanAndInvalidateCPUDataCache();

}

#define VOLUME_MIN -730

/* Register addresses as per datasheet */
#define RESET       0x00
#define PWRMGMT1    0x01
#define PWRMGMT2    0x02
#define PWRMGMT3    0x03
#define AINTFCE1    0x04
#define AINTFCE2    0x05
#define CLOCKIN1    0x06
#define CLOCKIN2    0x07
#define AINTFCE3    0x08
#define AINTFCE4    0x09
#define DACCTRL     0x0a
#define LDACVOL     0x0b
#define RDACVOL     0x0c
#define DST         0x0d
#define ADCCTL      0x0e
#define LADCVOL     0x0f
#define RADCVOL     0x10
#define GPIOCTL1    0x12
#define GPIO12      0x13
#define GPIO34      0x14
#define GPIO56      0x15
#define GPIOCTL2    0x16
#define GPIOPOL     0x17
#define LLINPUT12V  0x18
#define LLINPUT34V  0x19
#define RLINPUT12V  0x1a
#define RLINPUT34V  0x1b
#define LOUTVOL     0x1c
#define ROUTVOL     0x1d
#define LINOUTVOL   0x1e
#define OUT34VOL    0x1f
#define LOPGAVOL    0x20
#define ROPGAVOL    0x21
#define SPKRVOL     0x22
#define CLASSD1     0x23
#define CLASSD3     0x25
#define INMIXR1     0x27
#define INMIXR2     0x28
#define INMIXR3     0x29
#define INMIXR4     0x2a
#define INMIXR5     0x2b
#define INMIXR6     0x2c
#define OUTMIXR1    0x2d
#define OUTMIXR2    0x2e
#define OUTMIXR3    0x2f
#define OUTMIXR4    0x30
#define OUTMIXR5    0x31
#define OUTMIXR6    0x32
#define OUT34MIXR   0x33
#define LINMIXR1    0x34
#define LINMIXR2    0x35
#define SPKRMIXR    0x36
#define ADNLCTRL    0x37
#define ANTIPOP1    0x38
#define ANTIPOP2    0x39
#define MICBIAS     0x3a
#define PLL1        0x3c
#define PLL2        0x3d
#define PLL3        0x3e



#define EQ_GAIN_MASK       0x001f
#define EQ_CUTOFF_MASK     0x0060
#define EQ_GAIN_VALUE(x)   (((-x) + 12) & 0x1f)
#define EQ_CUTOFF_VALUE(x) ((((x) - 1) & 0x03) << 5)


const struct sound_settings_info audiohw_settings[] = {
    [SOUND_VOLUME]        = {"dB", 0,  1, -57,   6, -25},
    [SOUND_BASS]          = {"dB", 0,  1, -12,  12,   0},
    [SOUND_TREBLE]        = {"dB", 0,  1, -12,  12,   0},
    [SOUND_BALANCE]       = {"%",  0,  1,-100, 100,   0},
    [SOUND_CHANNELS]      = {"",   0,  1,   0,   5,   0},
    [SOUND_STEREO_WIDTH]  = {"%",  0,  5,   0, 250, 100},
#ifdef HAVE_RECORDING
    [SOUND_LEFT_GAIN]     = {"dB", 1,  1,-128,  96,   0},
    [SOUND_RIGHT_GAIN]    = {"dB", 1,  1,-128,  96,   0},
    [SOUND_MIC_GAIN]      = {"dB", 1,  1,-128, 108,  16},
#endif
    [SOUND_BASS_CUTOFF]   = {"",   0,  1,   1,   4,   1},
    [SOUND_TREBLE_CUTOFF] = {"",   0,  1,   1,   4,   1},
};

/* shadow registers */
unsigned int eq1_reg;
unsigned int eq5_reg;

/* convert tenth of dB volume (-73..6) to master volume register value */
int tenthdb2master(int db)
{
    /* +6 to -73db in 1dB steps == 79 levels = 7 bits */
    /* 1111111 == +6dB  (0x7f) = 127) */
    /* 1111001 == 0dB   (0x79) = 121) */
    /* 1100001 == -72dB (0x31) */
    /* 0110000 == -73dB (0x30) */
    /* 0101111 == Mute (0x2f) */

    /* 0000000 == Mute (0x0) */

    if (db < VOLUME_MIN) {
        return 0x0;
    } else {
        return((db/10)+121);
    }
}

/* Silently enable / disable audio output */
void audiohw_preinit(void)
{
    wmcodec_write(RESET,    0x0);    /* Reset */

    wmcodec_write(AINTFCE4, 0x8040); 

    wmcodec_write(GPIO12,0x1001);

    wmcodec_write(PWRMGMT1, 0x7);   /* BIASEN = 1, PLLEN = 1, BUFIOEN = 1, VMIDSEL = 1 */
    wmcodec_write(PWRMGMT2, 0x6800);

    wmcodec_write(DACCTRL,  0x0);

    wmcodec_write(LOUTVOL, 0x50 | (1<<8));
    wmcodec_write(ROUTVOL, 0x50 | (1<<8));
#if 0
    wmcodec_write(LOUT2VOL, 0xb9);
    wmcodec_write(ROUT2VOL, 0x1b9);

    wmcodec_write(BIASCTL,  0x100); /* BIASCUT = 1 */

    wmcodec_write(PWRMGMT1, 0x2d);   /* BIASEN = 1, PLLEN = 1, BUFIOEN = 1, VMIDSEL = 1 */
    wmcodec_write(PWRMGMT2, 0x180);
    wmcodec_write(PWRMGMT3, 0x6f);
   
    wmcodec_write(AINTFCE, 0x10);   /* 16-bit, I2S format */

    wmcodec_write(COMPAND, 0x0);
    wmcodec_write(CLKGEN, 0x14d);
    wmcodec_write(SRATECTRL, 0x0);
    wmcodec_write(GPIOCTL, 0x0);
    wmcodec_write(JACKDETECT0, 0x0);

    wmcodec_write(DACCTRL,  0x3);
    wmcodec_write(LDACVOL,  0xff);
    wmcodec_write(RDACVOL,  0x1ff);

    wmcodec_write(JACKDETECT1, 0x0);

    wmcodec_write(ADCCTL, 0x0);
    wmcodec_write(LADCVOL, 0xff);
    wmcodec_write(RADCVOL, 0xff);

    wmcodec_write(EQ1, 0x12c);
    wmcodec_write(EQ2, 0x2c);
    wmcodec_write(EQ3, 0x2c);
    wmcodec_write(EQ4, 0x2c);
    wmcodec_write(EQ5, 0x2c);

    wmcodec_write(DACLIMIT1, 0x32);
    wmcodec_write(DACLIMIT2, 0x0);

    wmcodec_write(NOTCH1, 0x0);
    wmcodec_write(NOTCH2, 0x0);
    wmcodec_write(NOTCH3, 0x0);
    wmcodec_write(NOTCH4, 0x0);

    wmcodec_write(PLLN, 0xa);
   
    wmcodec_write(PLLK1, 0x1);
    wmcodec_write(PLLK2, 0x1fd);
    wmcodec_write(PLLK3, 0x1e8);

    wmcodec_write(THREEDCTL, 0x0);
    wmcodec_write(OUT4ADC, 0x0);
    wmcodec_write(BEEPCTRL, 0x0);

    wmcodec_write(INCTRL, 0x0);
    wmcodec_write(LINPGAGAIN, 0x40);
    wmcodec_write(RINPGAGAIN, 0x140);

    wmcodec_write(LADCBOOST, 0x0);
    wmcodec_write(RADCBOOST, 0x0);

    wmcodec_write(OUTCTRL,  0x186);   /* Thermal shutdown, DACL2RMIX = 1, DACR2LMIX = 1, SPKBOOST = 1 */
    wmcodec_write(LOUTMIX, 0x15);
    wmcodec_write(ROUTMIX, 0x15);

    wmcodec_write(OUT3MIX,  0x40);
    wmcodec_write(OUT4MIX,  0x40);

    wmcodec_write(WMREG_3E, 0x8c90);
#endif
}

void audiohw_mute(int mute)
{
    if (mute)
    {
        /* Set DACMU = 1 to soft-mute the audio DACs. */
    	wmcodec_write(DACCTRL, 0x4);
    } else {
        /* Set DACMU = 0 to soft-un-mute the audio DACs. */
    	wmcodec_write(DACCTRL, 0x0);
    }
}

void audiohw_postinit(void)
{
    audiohw_mute(0);
}

void audiohw_set_headphone_vol(int vol_l, int vol_r)
{
    /* OUT1 */
    wmcodec_write(LOUTVOL, 0x180 | vol_l);
    wmcodec_write(ROUTVOL, 0x180 | vol_r);
}

void audiohw_set_lineout_vol(int vol_l, int vol_r)
{
    /* OUT2 */
    wmcodec_write(LINOUTVOL, vol_l);
    //wmcodec_write(ROUT2VOL, 0x180 | vol_r);
}

void audiohw_set_aux_vol(int vol_l, int vol_r)
{
    /* OUTMIX */
    wmcodec_write(OUTMIXR5, (vol_l << 5) );
    wmcodec_write(OUTMIXR6, (vol_r << 5) );
}

void audiohw_set_bass(int value)
{
    //eq1_reg = (eq1_reg & ~EQ_GAIN_MASK) | EQ_GAIN_VALUE(value);
    //wmcodec_write(EQ1, 0x100 | eq1_reg);
}

void audiohw_set_bass_cutoff(int value)
{
    //eq1_reg = (eq1_reg & ~EQ_CUTOFF_MASK) | EQ_CUTOFF_VALUE(value);
    //wmcodec_write(EQ1, 0x100 | eq1_reg);
}

void audiohw_set_treble(int value)
{
    //eq5_reg = (eq5_reg & ~EQ_GAIN_MASK) | EQ_GAIN_VALUE(value);
    //wmcodec_write(EQ5, eq5_reg);
}

void audiohw_set_treble_cutoff(int value)
{
    //eq5_reg = (eq5_reg & ~EQ_CUTOFF_MASK) | EQ_CUTOFF_VALUE(value);
    //wmcodec_write(EQ5, eq5_reg);
}

/* Nice shutdown of WM8985 codec */
void audiohw_close(void)
{
    audiohw_mute(1);

    wmcodec_write(PWRMGMT3, 0x0);

    wmcodec_write(PWRMGMT1, 0x0);

    wmcodec_write(PWRMGMT2, 0x40);
}

/* Note: Disable output before calling this function */
void audiohw_set_sample_rate(int fsel)
{
    /* Currently the WM8985 acts as slave to the SoC I2S controller, so no
       setup is needed here. This seems to be in contrast to every other WM
       driver in Rockbox, so this may need to change in the future. */
    (void)fsel;
}

#ifdef HAVE_RECORDING
void audiohw_enable_recording(bool source_mic)
{
    (void)source_mic; /* We only have a line-in (I think) */

    wmcodec_write(RESET, 0x0);    /*Reset*/

    wmcodec_write(PWRMGMT1, 0x12);
    wmcodec_write(PWRMGMT2, 0xbf3);  /* Enable ADC - 0x0c enables left/right PGA input, and 0x03 turns on power to the ADCs */
    wmcodec_write(PWRMGMT3, 0x1f3);

    wmcodec_write(AINTFCE1, 0x10);
    wmcodec_write(AINTFCE2, 0x1e);
    wmcodec_write(CLKCTRL1, 0x0);
    wmcodec_write(CLKCTRL2, 0x0);

    wmcodec_write(ADCCTL, 0x140);

    /* The iPod can handle multiple frequencies, but fix at 44.1KHz
       for now */
    audiohw_set_frequency(HW_FREQ_DEFAULT);

    //wmcodec_write(INCTRL,0x44);  /* Connect L2 and R2 inputs */

    /* Set L2/R2_2BOOSTVOL to 0db (bits 4-6) */
    /* 000 = disabled
       001 = -12dB
       010 = -9dB
       011 = -6dB
       100 = -3dB
       101 = 0dB
       110 = 3dB
       111 = 6dB
    */
    //wmcodec_write(LADCBOOST,0x50);
    //wmcodec_write(RADCBOOST,0x50);

    /* Set L/R input PGA Volume to 0db */
    //    wm8758_write(LINPGAVOL,0x3f);
    //    wm8758_write(RINPGAVOL,0x13f);

    /* Enable monitoring */
    //wmcodec_write(LOUTMIX,0x17); /* Enable output mixer - BYPL2LMIX @ 0db*/
    //wmcodec_write(ROUTMIX,0x17); /* Enable output mixer - BYPR2RMIX @ 0db*/

    audiohw_mute(0);
}

void audiohw_disable_recording(void) {
    audiohw_mute(1);

    wmcodec_write(PWRMGMT3, 0x0);

    wmcodec_write(PWRMGMT1, 0x0);

    wmcodec_write(PWRMGMT2, 0x40);
}

void audiohw_set_recvol(int left, int right, int type) {

    (void)left;
    (void)right;
    (void)type;
}

void audiohw_set_monitor(bool enable) {

    (void)enable;
}
#endif /* HAVE_RECORDING */
