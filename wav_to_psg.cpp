/* Converts 16-bit mono wave file into SMS PSG sample

wav_to_psg [in.wav]

Reads in.wav, converts to PSG sample and writes to out.wav, psg.bin, and out.vgm.
Doesn't parse wave file, just assumes data starts at offset 0x30. You might have
to improve this.

Input file should be whatever sample rate you want to use.

Algorithm:

The PSG's DAC is non-linear, so there are many more levels close to zero than
close to maximum loudness. Most sounds also have more samples near zero, but
the samples go above and below zero, while the DAC can only output levels above
zero. Simply offsetting the input to half-way will result in awful sound, since
it puts it where the DAC has many fewer levels.

The first step is to add a varying offset to the input samples so that they
are *always* positive, and usually close to zero. This makes a huge difference.
If you set make_positive_only to 1, you examine the result of this step only.

The conversion step simply finds the closest DAC level to this always-positive
sound wave. It can optionally add dither, but that makes little improvement.

Shay Green <gblargg@gmail.com> */

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


/**** Adjustable parameters/flags ****/

/* Controls how much bass is removed in make_positive(). Higher values remove less. */
int const high_pass_shift = 6;

/* Set to 1 to enable full dithering, which is noisy */
int const full_dither_enabled = 0;

/* Adjusts mild dither added all the time. */
double const mild_dither_level = 0.01;

/* Affects output rate of VGM file. Effective output rate is 44100 Hz / sample_repeat.
For example, a value of 4 results in a sample rate of 11025 Hz. */
int const sample_repeat = 4;

/* Set to 1 to only make input samples positive, then write them. */
int const make_positive_only = 0;

/* Set to 1 to record the offset to keep the waveform positive.
Useful to understand how this part of the algorithm works. */
int const make_offset_only = 0;

/* Non-linearity of DAC */
double const db_per_dac_step = 2.0;


/**** General utilities ****/

typedef signed char int8_t; /* 8-bit signed */
typedef unsigned char uint8_t; /* 8-bit unsigned */

/* Gets signed 16-bit little endian integer */
static unsigned get_le16s(uint8_t const* p)
{
	return (int8_t) p[1] * 0x100 + p[0];
}

/* Sets 16-bit little endian integer */
static void set_le16(uint8_t* p, unsigned n)
{
	p[0] = (uint8_t) (n);
	p[1] = (uint8_t) (n >> 8);
}

/* Prints error and exits */
static void error(const char* str)
{
	fprintf(stderr, "Error: %s\n", str);
	exit(EXIT_FAILURE);
}

/* Same as malloc(), but exits if allocation fails */
static void* malloc_chk(size_t n)
{
	void* p = malloc(n);
	if (!p)
	{
		error("Out of memory");
	}
	return p;
}

/* Reads file, sets *size_out to file size, and returns pointer to data */
static uint8_t* read_file(const char* path, long* size_out)
{
	uint8_t* data;
	FILE* in = fopen(path, "rb");
	if (!in)
	{
		error("Couldn't open file");
	}

	fseek(in, 0, SEEK_END);
	*size_out = ftell(in);

	data = (uint8_t*) malloc_chk(*size_out);

	rewind(in);
	if (fread(data, *size_out, 1, in) < 1)
	{
		error("Couldn't read file data");
	}

	fclose(in);

	return data;
}

/* Writes file */
static void write_file(void const* in, long size, const char* out_path)
{
	FILE* out = fopen(out_path, "wb");
	if (!out)
	{
		error("Couldn't open output file");
	}

	if (fwrite(in, size, 1, out) < 1)
	{
		error("Couldn't write data");
	}
	fclose(out);
}


/**** Step 1: Eliminate negative samples ****/

/* Makes all samples positive by adding varying offset to them */
static void make_positive(uint8_t* io, int size)
{
	int* fwd = (int*) malloc_chk(size * sizeof *fwd);
	int i;
	int min;

	/* Go forward and generate minimums with smooth return to zero */
	min = 0;
	for (i = 0; i < size; i++)
	{
		int s = get_le16s(&io[i * 2]);
		if (min > s)
		{
			min = s;
		}
		fwd[i] = min;
		min -= min >> high_pass_shift;
	}

	/* Go backward to get smooth return to zero in opposite direction,
	then add smoothed minimum as offset, resulting in no negative samples */
	min = 0;
	for (i = size; --i >= 0;)
	{
		int s = get_le16s(&io[i * 2]);

		int f = fwd[i];
		if (min > f)
		{
			min = f;
		}

		s -= min;
		if (s > 0x7FFF)
		{
			s = 0x7FFF;
		}
		assert( s >= 0 );

		if (make_offset_only)
		{
			s = -min;
		}

		set_le16(&io[i * 2], s);
		min -= min >> high_pass_shift;
	}

	free(fwd);
}


/**** Step 2: PCM to PSG ****/

/* We don't use first and last DAC levels (see init_dac_table()) */
enum
{
	dac_steps = 14
};

/* Output level for each DAC value, from 0.0 to 1.0 */
static double dac_table [dac_steps + 1]; /* extra entry simplifies code */

/* DAC level, from 0.0 to 1.0 */
static double calc_dac_level(int i)
{
	return pow(10.0, db_per_dac_step / 20.0 * (i - 15));
}

static void init_dac_table()
{
	/* Zero DAC level isn't useful since it is a big jump to 1, as compared */
	/* to the jump from 1 to 2. */
	double offset = calc_dac_level(1);

	/* Final DAC level also isn't useful since it's too large a jump */
	double scale = 1.0 / (calc_dac_level(dac_steps) - offset);

	/* Generate table of DAC levels */
	for (int i = 1; i < dac_steps; i++)
	{
		dac_table[i] = (calc_dac_level(i + 1) - offset) * scale;
	}

	/* Entry past end should be really close to last entry. Making it exactly
	the same results in divide by zero in pcm_to_psg() since range would be 0. */
	dac_table[dac_steps] = dac_table[dac_steps - 1] + 0.000001;
}

/* Converts PCM to PSG DAC values */
static void pcm_to_psg(uint8_t const* in, int size, uint8_t* out)
{
	for (int i = size; --i >= 0;)
	{
		/* Get source sample, which should now be completely positive. */
		double s = 1.0 / 0x7FFF * get_le16s(&in[i * 2]);
		assert( s >= 0 && s <= 1.0 );

		/* triangular probability distribution */
		s += mild_dither_level / RAND_MAX * rand() - mild_dither_level / 2;
		s += mild_dither_level / RAND_MAX * rand() - mild_dither_level / 2;
		if (s < 0)
		{
			s = 0;
		}
		if (s > 1)
		{
			s = 1;
		}

		/* Find closest DAC level that's lower than sample */
		int index = dac_steps - 1;
		while (dac_table[index] > s)
			index--;
		assert( index >= 0 );

		{
			/* Find where it lies between DAC level and next higher level, 
			where 0.0 = DAC level, 0.5 = middle, 1.0 = next higher level */
			double range = dac_table[index + 1] - dac_table[index];
			double fract = (s - dac_table[index]) / range;

			/* Use fraction to determine whether to round up to next DAC level */
			if (full_dither_enabled)
			{
				/* For dither case, use fraction as probability that next 
				higher DAC level is used */
				if (rand() > RAND_MAX * fract)
				{
					index++;
				}
			}
			else if (fract > 0.5) /* round to closest */
			{
				index++;
			}
		}

		out[i] = 14 - index;
	}
}


/**** Output conversion ****/

static void psg_to_raw(uint8_t const* in, int size, const char* out_path)
{
	int i;
	FILE* out = fopen(out_path, "wb");
	if (!out)
	{
		error("Couldn't create output file");
	}

	for (i = 0; i < size; i += 2)
	{
		fputc(in[i] + 0x10 * in[i + 1], out);
	}

	fclose(out);
}

static void psg_to_pcm(uint8_t const* in, int size, uint8_t* out)
{
	int i;
	for (i = 0; i < size; i++)
	{
		int index = 14 - in[i];
		int s = (int) (dac_table[index] * 0x7FFF);
		set_le16(&out[i * 2], s);
	}
}

enum
{
	cmd_gg_stereo = 0x4F,
	cmd_psg = 0x50,
	cmd_short_delay = 0x70
};

/* Outputs PSG data to VGM file */
static void psg_to_vgm(uint8_t const* in, int size, const char* out_path)
{
	FILE* out = fopen(out_path, "wb");
	if (!out)
	{
		error("Couldn't create output file");
	}

	{
		int i;
		char header [0x40] = "Vgm " "\0\0\0\0" "\0\1\0\0" "\x99\x9E\x36\x00";
		fwrite(header, sizeof header, 1, out);

		putc(cmd_gg_stereo, out);
		putc(0xFF, out);
		putc(cmd_psg, out);
		putc(0x80, out);
		putc(cmd_psg, out);
		putc(0x00, out);

		for (i = 0; i < size; i++)
		{
			putc(cmd_psg, out);
			putc(0x90 + in[i], out);
			putc(cmd_short_delay + sample_repeat - 1, out);
		}
	}

	fclose(out);
}

int main(int argc, char** argv)
{
	/* Read input wave file, which must be 16-bit mono */
	long size;
	uint8_t* data = read_file((argc > 1 ? argv[1] : "in.wav"), &size);

	/* Just assume that wave data is at offset 0x30, which might skip a few
	samples at beginning */
	int const offset = 0x30;
	uint8_t* samples = data + offset;
	int const samples_size = (size - offset) / 2;

	make_positive(samples, samples_size);

	if (!make_positive_only)
	{
		uint8_t* psg_data = (uint8_t*) malloc_chk(samples_size);
		init_dac_table();
		pcm_to_psg(samples, samples_size, psg_data);
		psg_to_pcm(psg_data, samples_size, samples);
		psg_to_vgm(psg_data, samples_size, "vgm");
		psg_to_raw(psg_data, samples_size, "psg.bin");
		free(psg_data);
	}

	write_file(data, size, "out.wav");

	return 0;
}
