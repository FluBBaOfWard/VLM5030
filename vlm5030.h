#ifndef VLM5030_h
#define VLM5030_h

#define K_SIZE (10)

struct VLM5030interface
{
	int memory_region;  /* memory region of speech rom    */
	int memory_size;    /* memory size of speech rom (0=memory region length) */
};

struct vlm5030_info
{
	const struct VLM5030interface *intf;
	
	/* need to save state */
	
	const unsigned char *rom;
	int address_mask;
	unsigned short address;
	bool pin_BSY;
	bool pin_ST;
	bool pin_VCU;
	bool pin_RST;
	unsigned char latch_data;
	unsigned short vcu_addr_h;
	unsigned char parameter;
	unsigned char phase;
	
	/* state of option paramter */
	int frame_size;
	int pitch_offset;
	unsigned char interp_step;
	
	unsigned char interp_count;       /* number of interp periods    */
	unsigned char sample_count;       /* sample number within interp */
	unsigned char pitch_count;
	
	/* these contain data describing the current and previous voice frames */
	unsigned short old_energy;
	unsigned char old_pitch;
	short old_k[K_SIZE];
	unsigned short target_energy;
	unsigned char target_pitch;
	short target_k[K_SIZE];
	
	unsigned short new_energy;
	unsigned char new_pitch;
	short new_k[K_SIZE];
	
	/* these are all used to contain the current state of the sound generation */
	unsigned int current_energy;
	unsigned int current_pitch;
	int current_k[K_SIZE];
	
	int x[K_SIZE];
};


/* Allocate and init a chip */
void *vlm5030_start(const void *rom);

/* set speech rom address */
void VLM5030_set_rom(struct vlm5030_info *chip, const void *speech_rom, int size);

void vlm5030_update_callback(struct vlm5030_info *chip, short *buffer, int length);

/* get BSY pin level */
bool VLM5030_BSY(struct vlm5030_info *chip);
/* latch contoll data */
void VLM5030_WRITE8(struct vlm5030_info *chip, unsigned char data);
/* set RST pin level : reset / set table address A8-A15 */
void VLM5030_RST(struct vlm5030_info *chip, bool pin );
/* set VCU pin level : ?? unknown */
void VLM5030_VCU(struct vlm5030_info *chip, bool pin );
/* set ST pin level  : set table address A0-A7 / start speech */
void VLM5030_ST(struct vlm5030_info *chip, bool pin );

#endif

