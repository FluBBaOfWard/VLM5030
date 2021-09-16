/*
 vlm5030.c
 
 VLM5030 emulator
 
 Written by Tatsuyuki Satoh
 Based on TMS5220 simulator (tms5220.c)
 
 note:
 memory read cycle(==sampling rate) = 122.9u(440clock)
 interpolator (LC8109 = 2.5ms)      = 20 * samples(125us)
 frame time (20ms)                  =  4 * interpolator
 9bit DAC is composed of 5bit Physical and 3bitPWM.
 
 todo:
 Noise Generator circuit without 'mame_rand()' function.
 
 ----------- command format (Analytical result) ----------
 
 1)end of speech (8bit)
 :00000011:
 
 2)silent some frame (8bit)
 :????SS01:
 
 SS : number of silent frames
 00 = 2 frame
 01 = 4 frame
 10 = 6 frame
 11 = 8 frame
 
 3)-speech frame (48bit)
 function:   6th  :  5th   :   4th  :   3rd  :   2nd  : 1st    :
 end     :   ---  :  ---   :   ---  :   ---  :   ---  :00000011:
 silent  :   ---  :  ---   :   ---  :   ---  :   ---  :0000SS01:
 speech  :11111122:22233334:44455566:67778889:99AAAEEE:EEPPPPP0:
 
 EEEEE  : energy : volume 0=off, 0x1f=max
 PPPPP  : pitch  : 0=noise, 1=fast, 0x1f=slow
 111111 : K1     : 48=off
 22222  : K2     : 0=off, 1=+min, 0x0f=+max, 0x10=off, 0x11=+max, 0x1f=-min
 : 16 == special function??
 3333   : K3     : 0=off, 1=+min, 0x07=+max, 0x08=-max, 0x0f=-min
 4444   : K4     :
 555    : K5     : 0=off, 1=+min, 0x03=+max, 0x04=-max, 0x07=-min
 666    : K6     :
 777    : K7     :
 888    : K8     :
 999    : K9     :
 AAA    : K10    :
 
 ---------- chirp table information ----------
 
 DAC PWM cycle == 88system clock , (11clock x 8 pattern) = 40.6KHz
 one chirp     == 5 x PWM cycle == 440systemclock(8,136Hz)
 
 chirp  0   : volume 10- 8 : with filter
 chirp  1   : volume  8- 6 : with filter
 chirp  2   : volume  6- 4 : with filter
 chirp  3   : volume   4   : no filter ??
 chirp  4- 5: volume  4- 2 : with filter
 chirp  6-11: volume  2- 0 : with filter
 chirp 12-..: vokume   0   : silent
 
 ---------- digial output information ----------
 when ME pin = high , some status output to A0..15 pins
 
 A0..8   : DAC output value (abs)
 A9      : DAC sign flag , L=minus,H=Plus
 A10     : energy reload flag (pitch pulse)
 A11..15 : unknown
 
 [DAC output value(signed 9bit)] = A9 ? A0..8 : -(A0..8)
 
 */
#include <nds.h>
#include <stdlib.h>
#include <string.h>
#include "vlm5030.h"

bool vlm5030ClockNoise(void);

/* interpolator per frame   */
#define FR_SIZE 4
/* samples per interpolator */
#define IP_SIZE_SLOWER  (240/FR_SIZE)
#define IP_SIZE_SLOW    (200/FR_SIZE)
#define IP_SIZE_NORMAL  (160/FR_SIZE)
#define IP_SIZE_FAST    (120/FR_SIZE)
#define IP_SIZE_FASTER  ( 80/FR_SIZE)

/* phase value */
enum {
	PH_RESET,
	PH_IDLE,
	PH_SETUP,
	PH_WAIT,
	PH_RUN,
	PH_STOP,
	PH_END
};

/*
 speed parameter
 SPC SPB SPA
 1   0   1  more slow (05h)     : 42ms   (150%) : 60sample
 1   1   x  slow      (06h,07h) : 34ms   (125%) : 50sample
 x   0   0  normal    (00h,04h) : 25.6ms (100%) : 40sample
 0   0   1  fast      (01h)     : 20.2ms  (75%) : 30sample
 0   1   x  more fast (02h,03h) : 12.2ms  (50%) : 20sample
 */
static const int VLM5030_speed_table[8] =
{
	IP_SIZE_NORMAL,
	IP_SIZE_FAST,
	IP_SIZE_FASTER,
	IP_SIZE_FASTER,
	IP_SIZE_NORMAL,
	IP_SIZE_SLOWER,
	IP_SIZE_SLOW,
	IP_SIZE_SLOW
};

/* ROM Tables */

/* This is the energy lookup table */
static const unsigned short energytable[0x20] = {
	0,  1,  2,  3,  5,  6,  7,  9, /*  0-7  */
	11, 13, 15, 17, 19, 22, 24, 27, /*  8-15 */
	31, 34, 38, 42, 47, 51, 57, 62, /* 16-23 */
	68, 75, 82, 89, 98,107,116,127  /* 24-31 */
};

/* This is the pitch lookup table */
static const unsigned short pitchtable[0x20] = {
	0,                               /* 0     : random mode */
	21,                              /* 1     : start=21    */
	22, 23, 24, 25, 26, 27, 28, 29,  /*  2- 9 : 1step       */
	31, 33, 35, 37, 39, 41, 43, 45,  /* 10-17 : 2step       */
	49, 53, 57, 61, 65, 69, 73, 77,  /* 18-25 : 4step       */
	85, 93, 101,109,117,125          /* 26-31 : 8step       */
};

static const int K1_table[0x40] = {
	390, 403, 414, 425, 434, 443, 450, 457,
	463, 469, 474, 478, 482, 485, 488, 491,
	494, 496, 498, 499, 501, 502, 503, 504,
	505, 506, 507, 507, 508, 508, 509, 509,
	-390,-376,-360,-344,-325,-305,-284,-261,
	-237,-211,-183,-155,-125, -95, -64, -32,
	0,  32,  64,  95, 125, 155, 183, 211,
	237, 261, 284, 305, 325, 344, 360, 376
};
static const int K2_table[0x20] = {
	0,  50, 100, 149, 196, 241, 284, 325,
	362, 396, 426, 452, 473, 490, 502, 510,
	0,-510,-502,-490,-473,-452,-426,-396, /* entry 16(0x10) either has some special function, purpose unknown, or is a manufacturing error and should have been -512 */
	-362,-325,-284,-241,-196,-149,-100, -50
};
static const int K3_table[0x10] = {
	0, 64, 128, 192, 256, 320, 384, 448,
	-512,-448,-384,-320,-256,-192,-128, -64
};
static const int K5_table[0x08] = {
	0, 128, 256, 384,-512,-384,-256,-128
};

static void VLM5030_reset(struct vlm5030_info *chip);
static void VLM5030_update(struct vlm5030_info *chip);
static void VLM5030_setup_parameter(struct vlm5030_info *chip, unsigned char param);



/* start VLM5030 with sound rom              */
/* speech_rom == 0 -> use sampling data mode */
void *vlm5030_start(const void *rom)
{
	struct vlm5030_info *chip;

	chip = malloc(sizeof(*chip));
	memset(chip, 0, sizeof(*chip));

	/* reset input pins */
	chip->pin_RST = false;
	chip->pin_ST = false;
	chip->pin_VCU = false;
	chip->latch_data = 0;

	VLM5030_reset(chip);
	chip->phase = PH_IDLE;

	chip->rom = rom;

	/* memory size */
	chip->address_mask = 0x10000-1;

	return chip;
}

static void VLM5030_reset(struct vlm5030_info *chip)
{
	chip->phase = PH_RESET;
	chip->address = 0;
	chip->vcu_addr_h = 0;
	chip->pin_BSY = false;

	chip->old_energy = 0;
	chip->old_pitch = 0;
	chip->new_energy = 0;
	chip->new_pitch = 0;
	chip->current_energy = 0;
	chip->current_pitch = 0;
	chip->target_energy = 0;
	chip->target_pitch = 0;
	memset(chip->old_k, 0, sizeof(chip->old_k));
	memset(chip->new_k, 0, sizeof(chip->new_k));
	memset(chip->current_k, 0, sizeof(chip->current_k));
	memset(chip->target_k, 0, sizeof(chip->target_k));
	chip->interp_count = chip->sample_count = chip->pitch_count = 0;
	memset(chip->x, 0, sizeof(chip->x));
	/* reset parameters */
	VLM5030_setup_parameter(chip, 0x00);
}

static int get_bits(struct vlm5030_info *chip, int sbit, int bits)
{
	int offset = chip->address + (sbit>>3);
	int data;

	data = chip->rom[offset&chip->address_mask] +
	(((int)chip->rom[(offset+1)&chip->address_mask])<<8);
	data >>= (sbit&7);
	data &= (0xff>>(8-bits));

	return data;
}

/* get next frame */
static int parse_frame(struct vlm5030_info *chip)
{
	unsigned char cmd;
	int i;

	/* remember previous frame */
	chip->old_energy = chip->new_energy;
	chip->old_pitch = chip->new_pitch;
	for ( i=0; i<K_SIZE; i++ ) {
		chip->old_k[i] = chip->new_k[i];
	}

	/* command byte check */
	cmd = chip->rom[chip->address&chip->address_mask];
	if ( cmd & 0x01 ) {
		/* extend frame */
		chip->new_pitch = 0;
		chip->new_energy = 0;
		for ( i=0; i<K_SIZE; i++ ) {
			chip->new_k[i] = 0;
		}
		chip->address++;
		if ( cmd & 0x02 ) {
			/* end of speech */
			/* logerror("VLM5030 %04X end \n",chip->address ); */
			return 0;
		}
		else {
			/* silent frame */
			int nums = ( (cmd>>2)+1 )*2;
			/* logerror("VLM5030 %04X silent %d frame\n",chip->address,nums ); */
			return nums * FR_SIZE;
		}
	}
	/* pitch */
	chip->new_pitch = ( pitchtable[get_bits(chip, 1,5)] + chip->pitch_offset )&0xff;
	/* energy */
	chip->new_energy = energytable[get_bits(chip, 6,5)];

	/* 10 K's */
	chip->new_k[9] = K5_table[get_bits(chip,11,3)];
	chip->new_k[8] = K5_table[get_bits(chip,14,3)];
	chip->new_k[7] = K5_table[get_bits(chip,17,3)];
	chip->new_k[6] = K5_table[get_bits(chip,20,3)];
	chip->new_k[5] = K5_table[get_bits(chip,23,3)];
	chip->new_k[4] = K5_table[get_bits(chip,26,3)];
	chip->new_k[3] = K3_table[get_bits(chip,29,4)];
	chip->new_k[2] = K3_table[get_bits(chip,33,4)];
	chip->new_k[1] = K2_table[get_bits(chip,37,5)];
	chip->new_k[0] = K1_table[get_bits(chip,42,6)];

	chip->address += 6;
//	logerror("VLM5030 %04X voice \n",chip->address );
	return FR_SIZE;
}

/* realtime update */
static void VLM5030_update(struct vlm5030_info *chip)
{
	vlm5030_update_callback(chip, 0, 0);
//	m_channel->update();
}

/* setup parameter option when RST=H */
static void VLM5030_setup_parameter(struct vlm5030_info *chip, unsigned char param)
{
	/* latch parameter value */
	chip->parameter = param;

	/* bit 0,1 : 4800bps / 9600bps , interpolator step */
	if ( param & 2 ) { /* bit 1 = 1 , 9600bps */
		chip->interp_step = 4; /* 9600bps : no interpolator */
	}
	else if ( param & 1 ) { /* bit1 = 0 & bit0 = 1 , 4800bps */
		chip->interp_step = 2; /* 4800bps : 2 interpolator */
	}
	else { /* bit1 = bit0 = 0 : 2400bps */
		chip->interp_step = 1; /* 2400bps : 4 interpolator */
	}

	/* bit 3,4,5 : speed (frame size) */
	chip->frame_size = VLM5030_speed_table[(param>>3) &7];

	/* bit 6,7 : low / high pitch */
	if ( param & 0x80 ) { /* bit7=1 , high pitch */
		chip->pitch_offset = -8;
	}
	else if ( param & 0x40 ) { /* bit6=1 , low pitch */
		chip->pitch_offset = 8;
	}
	else {
		chip->pitch_offset = 0;
	}
}


void VLM5030_restore_state(struct vlm5030_info *chip)
{
	int i;

	int interp_effect = FR_SIZE - (chip->interp_count%FR_SIZE);
	/* restore parameter data */
	VLM5030_setup_parameter(chip, chip->parameter);

	/* restore current energy, pitch & filter */
	chip->current_energy = chip->old_energy + (chip->target_energy - chip->old_energy) * interp_effect / FR_SIZE;
	if ( chip->old_pitch > 1 ) {
		chip->current_pitch = chip->old_pitch + (chip->target_pitch - chip->old_pitch) * interp_effect / FR_SIZE;
	}
	for ( i = 0; i < K_SIZE ; i++ ) {
		chip->current_k[i] = chip->old_k[i] + (chip->target_k[i] - chip->old_k[i]) * interp_effect / FR_SIZE;
	}
}

/* set speech rom address */
void VLM5030_set_rom(struct vlm5030_info *chip, const void *speech_rom, int size)
{
	chip->rom = speech_rom;
	chip->address_mask = size-1;
}

/* get BSY pin level */
bool VLM5030_BSY(struct vlm5030_info *chip)
{
	VLM5030_update(chip);
//	return chip->pin_BSY;
	return (chip->phase == PH_SETUP || chip->phase == PH_RUN);
}

/* latch control data */
void VLM5030_WRITE8(struct vlm5030_info *chip, unsigned char data)
{
	chip->latch_data = data;
}

/* set RST pin level : reset / set table address A8-A15 */
void VLM5030_RST(struct vlm5030_info *chip, bool pin )
{
	if ( chip->pin_RST != pin ) {
		chip->pin_RST = pin;
		if ( !pin ) {
			/* H -> L : latch parameters */
			VLM5030_setup_parameter(chip, chip->latch_data);
		}
		else {
			/* L -> H : reset chip */
			if ( chip->pin_BSY ) {
				VLM5030_reset(chip);
			}
		}
	}
}

/* set VCU pin level : ?? unknown */
void VLM5030_VCU(struct vlm5030_info *chip, bool pin)
{
	/* direct mode / indirect mode */
	chip->pin_VCU = pin;
}

/* set ST pin level : set table address A0-A7 / start speech */
void VLM5030_ST(struct vlm5030_info *chip, bool pin )
{
	int table;

	if ( chip->pin_ST != pin ) {
		chip->pin_ST = pin;
		/* pin level is changed */
		if ( !pin ) {
			/* H -> L */
			if( chip->pin_VCU ) {
				/* direct access mode & address High */
				chip->vcu_addr_h = ((int)chip->latch_data<<8) + 0x01;
			}
			else {
				/* start speech */
				/* check access mode */
				if ( chip->vcu_addr_h ) {
					/* direct access mode */
					chip->address = (chip->vcu_addr_h&0xff00) + chip->latch_data;
					chip->vcu_addr_h = 0;
				}
				else {
					/* indirect access mode */
					table = (chip->latch_data&0xfe) + (((int)chip->latch_data&1)<<8);
					chip->address = (((int)chip->rom[table&chip->address_mask])<<8)
					|        chip->rom[(table+1)&chip->address_mask];
#if 0
					/* show unsupported parameter message */
					if( chip->interp_step != 1) {
						popmessage("No %d %dBPS parameter", table/2, chip->interp_step*2400);
					}
#endif
				}
				VLM5030_update(chip);
				/* logerror("VLM5030 %02X start adr=%04X\n",table/2,chip->address ); */
				/* reset process status */
				chip->sample_count = chip->frame_size;
				chip->interp_count = FR_SIZE;
				/* clear filter */
				/* start after 3 sampling cycle */
				chip->phase = PH_RUN;
			}
		}
		else {
			/* L -> H */
			/* setup speech , BSY on after 30ms? */
			chip->phase = PH_SETUP;
			chip->sample_count = 1; /* wait time for busy on */
			chip->pin_BSY = true;
		}
	}
}

/* decode and buffering data */
void vlm5030_update_callback(struct vlm5030_info *chip, short *buffer, int length)
{
	int buf_count = 0;
	int interp_effect;
	int i;
	int u[11];

	/* running */
	if ( chip->phase == PH_RUN || chip->phase == PH_STOP ) {
		/* playing speech */
		while ( length > 0 ) {
			int current_val;

			/* check new interpolator or new frame */
			if ( chip->sample_count == 0 ) {
				if ( chip->phase == PH_STOP ) {
					chip->phase = PH_END;
					chip->sample_count = 1;
					goto phase_stop; /* continue to end phase */
				}
				chip->sample_count = chip->frame_size;
				/* interpolator changes */
				if ( chip->interp_count == 0 ) {
					/* change to new frame */
					chip->interp_count = parse_frame(chip); /* with change phase */
					if ( chip->interp_count == 0 ) {
						/* end mark found */
						chip->interp_count = FR_SIZE;
						chip->sample_count = chip->frame_size; /* end -> stop time */
						chip->phase = PH_STOP;
					}
					/* Set old target as new start of frame */
					chip->current_energy = chip->old_energy;
					chip->current_pitch = chip->old_pitch;
					for ( i=0; i<K_SIZE; i++ ) {
						chip->current_k[i] = chip->old_k[i];
					}
					/* is this a zero energy frame? */
					if ( chip->current_energy == 0 ) {
						chip->target_energy = 0;
						chip->target_pitch = chip->current_pitch;
						for ( i=0; i<K_SIZE; i++ ) {
							chip->target_k[i] = chip->current_k[i];
						}
					}
					else {
						chip->target_energy = chip->new_energy;
						chip->target_pitch = chip->new_pitch;
						for ( i=0; i<K_SIZE; i++ ) {
							chip->target_k[i] = chip->new_k[i];
						}
					}
				}
				/* next interpolator */
				/* Update values based on step values 25% , 50% , 75% , 100% */
				chip->interp_count -= chip->interp_step;
				/* 3,2,1,0 -> 1,2,3,4 */
				interp_effect = FR_SIZE - (chip->interp_count%FR_SIZE);
				chip->current_energy = chip->old_energy + (chip->target_energy - chip->old_energy) * interp_effect / FR_SIZE;
				if ( chip->old_pitch > 1 ) {
					chip->current_pitch = chip->old_pitch + (chip->target_pitch - chip->old_pitch) * interp_effect / FR_SIZE;
				}
				for ( i = 0; i < K_SIZE ; i++ ) {
					chip->current_k[i] = chip->old_k[i] + (chip->target_k[i] - chip->old_k[i]) * interp_effect / FR_SIZE;
				}
			}
			/* calculate digital filter */
			if ( chip->old_energy == 0 ) {
				/* generate silent samples here */
				current_val = 0x00;
			}
			else if ( chip->old_pitch <= 1 ) {
				/* generate unvoiced samples here */
				current_val = vlm5030ClockNoise() ? chip->current_energy : -chip->current_energy;
			}
			else {
				/* generate voiced samples here */
				current_val = ( chip->pitch_count == 0) ? chip->current_energy : 0;
			}

			/* Lattice filter here */
			u[10] = current_val;
			for ( i = 9; i >= 0; i-- ) {
				u[i] = u[i+1] - ((-chip->current_k[i] * chip->x[i]) / 512);
			}
			for ( i = 9; i >= 1; i-- ) {
				chip->x[i] = chip->x[i-1] + ((-chip->current_k[i-1] * u[i-1]) / 512);
			}
			chip->x[0] = u[0];

			/* clipping, buffering */
			if ( u[0] > 511 ) {
				buffer[buf_count] = 511<<6;
			}
			else if ( u[0] < -511 ) {
				buffer[buf_count] = -511<<6;
			}
			else {
				buffer[buf_count] = (u[0] << 6);
			}
			buf_count++;

			/* sample count */
			chip->sample_count--;
			/* pitch */
			chip->pitch_count++;
			if ( chip->pitch_count >= chip->current_pitch ) {
				chip->pitch_count = 0;
			}
			/* size */
			length--;
		}
/*      return;*/
	}
	/* stop phase */
phase_stop:
	switch( chip->phase ) {
		case PH_SETUP:
			if ( chip->sample_count <= length ) {
				chip->sample_count = 0;
				/* logerror("VLM5030 BSY=H\n" ); */
				/* pin_BSY = true; */
				chip->phase = PH_WAIT;
			}
			else {
				chip->sample_count -= length;
			}
			break;
		case PH_END:
			if ( chip->sample_count <= length ) {
				chip->sample_count = 0;
				/* logerror("VLM5030 BSY=L\n" ); */
				chip->pin_BSY = false;
				chip->phase = PH_IDLE;
			}
			else {
				chip->sample_count -= length;
			}
	}
	/* silent buffering */
	while ( length > 0 ) {
		buffer[buf_count++] = 0x00;
		length--;
	}
}
