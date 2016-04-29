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

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <algorithm>

// General utilities

// Helper class for file IO
class FileReader
{
	std::ifstream _f;
	uint32_t _size;

	static bool needSwap()
	{
		return (*(uint32_t*)"A   " == 0x41202020);
	}

public:
	explicit FileReader(const std::string& filename)
	{
		_f.exceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);
		_f.open(filename, std::ifstream::binary);
		_f.seekg(0, std::ios::end);
		_size = (uint32_t)(_f.tellg());
		_f.seekg(0, std::ios::beg);
	}

	~FileReader()
	{
		_f.close();
	}

	template<typename T>
	void read(std::vector<T>& dest, int count)
	{
		for (auto i = 0; i < count; ++i)
		{
			T t;
			_f.read((char*)&t, sizeof(t));
			dest.push_back(t);
		}
	}

	uint32_t read32()
	{
		uint32_t v;
		_f.read((char*)&v, sizeof(uint32_t));
		if (needSwap())
		{
			v = ((v >> 24) & 0x000000ff) | ((v >> 8) & 0x0000ff00) |
				((v << 8) & 0x00ff0000) | ((v << 24) & 0xff000000);
		}
		return v;
	}

	uint16_t read16()
	{
		uint16_t v;
		_f.read((char*)&v, sizeof(uint16_t));
		if (needSwap())
		{
			v = ((v << 8) & 0x00ff0000) | ((v << 24) & 0xff000000);
		}
		return v;
	}

	uint8_t read()
	{
		uint8_t v;
		_f.read((char*)&v, sizeof(uint8_t));
		return v;
	}
	/*
	FileReader& checkMarker(char marker[5])
	{
		if (read32() != str2ul(marker))
		{
			std::ostringstream ss;
			ss << "Marker " << marker << " not found in file";
			throw std::runtime_error(ss.str());
		}
		return *this;
	}
	*/

	uint32_t size() const
	{
		return _size;
	}

	FileReader& seek(int offset)
	{
		_f.seekg(offset, std::ios::cur);
		return *this;
	}
};

// Gets signed 16-bit little endian integer
static unsigned get_le16s(uint8_t const* p)
{
	return (int8_t) p[1] * 0x100 + p[0];
}

// Sets 16-bit little endian integer 
static void set_le16(uint8_t* p, unsigned n)
{
	p[0] = (uint8_t) (n);
	p[1] = (uint8_t) (n >> 8);
}

// Prints error and exits 
static void error(const char* str)
{
	fprintf(stderr, "Error: %s\n", str);
	exit(EXIT_FAILURE);
}

// Same as malloc(), but exits if allocation fails 
static void* malloc_chk(size_t n)
{
	void* p = malloc(n);
	if (!p)
	{
		error("Out of memory");
	}
	return p;
}

// Writes file 
void write_file(const std::vector<uint8_t>& header, const std::vector<int16_t>& samples, const std::string& filename)
{
	printf("Writing to %s...", filename.c_str());
	std::ofstream f;
	f.exceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);
	f.open(filename, std::ifstream::binary);
	std::copy(header.begin(), header.end(), std::ostreambuf_iterator<char>(f));
	f.write((const char*)samples.data(), samples.size() * sizeof(uint16_t));
	printf("done\n");
}


// *** Step 1: Eliminate negative samples ***

// Makes all samples positive by adding varying offset to them 
static void make_positive(std::vector<int16_t>& data, std::vector<int16_t>& offsets, int high_pass_shift)
{
	// Go forward and generate minimums with smooth return to zero 
	int16_t min = 0;
	for (auto sample: data)
	{
		// Find the max of the current value and the low-pass max signal
		min = std::min(min, sample);
		offsets.push_back(min);
		// Make the value decay to 0
		min -= min >> high_pass_shift;
	}

	// This will have produced steps downwards with slow return to 0,
	// so we go backwards and add smooth returns to 0 in the other direction.
	// and then scale and offset the sample to match
	for (int i = data.size() - 1; i >= 0; --i)
	{
		int16_t sample = data[i];

		min = std::min(min, sample);
		min = std::min(min, offsets[i]);
		offsets[i] = min == std::numeric_limits<int16_t>::min() ? std::numeric_limits<int16_t>::max() : -min;

		// Then offset the sample
		int adjustedSample = sample - min;
		// Every sample should be >0 now
		assert(adjustedSample >= 0);
		// Then scale down by half
		adjustedSample /= 2;

		data[i] = (int16_t)adjustedSample;
		min -= min >> high_pass_shift;
	}
}


// *** Step 2: PCM to PSG ***

// We don't use first and last DAC levels (see init_dac_table()) 
enum
{
	dac_steps = 14
};

// Output level for each DAC value, from 0.0 to 1.0 
static double dac_table [dac_steps + 1]; // extra entry simplifies code 

// DAC level, from 0.0 to 1.0 
static double calc_dac_level(int i, double db_per_dac_step)
{
	return pow(10.0, db_per_dac_step / 20.0 * (i - 15));
}

static void init_dac_table(double db_per_dac_step)
{
	// Zero DAC level isn't useful since it is a big jump to 1, as compared 
	// to the jump from 1 to 2. 
	double offset = calc_dac_level(1, db_per_dac_step);

	// Final DAC level also isn't useful since it's too large a jump 
	double scale = 1.0 / (calc_dac_level(dac_steps, db_per_dac_step) - offset);

	// Generate table of DAC levels 
	for (int i = 1; i < dac_steps; i++)
	{
		dac_table[i] = (calc_dac_level(i + 1, db_per_dac_step) - offset) * scale;
	}

	/* Entry past end should be really close to last entry. Making it exactly
	the same results in divide by zero in pcm_to_psg() since range would be 0. */
	dac_table[dac_steps] = dac_table[dac_steps - 1] + 0.000001;
}

// Converts PCM to PSG DAC values 
static void pcm_to_psg(const std::vector<int16_t>& in, std::vector<uint8_t>& out, double mild_dither_level, bool full_dither_enabled)
{
	out.resize(in.size());
	for (int i = in.size() - 1; i >= 0; --i)
	{
		// Get source sample, which should now be completely positive. 
		double s = 1.0 / 0x7FFF * in[i];
		assert( s >= 0 && s <= 1.0 );

		// triangular probability distribution 
		s += mild_dither_level / RAND_MAX * rand() - mild_dither_level / 2;
		s += mild_dither_level / RAND_MAX * rand() - mild_dither_level / 2;
		s = std::max(s, 0.0);
		s = std::min(s, 1.0);

		// Find closest DAC level that's lower than sample 
		int index = dac_steps - 1;
		while (dac_table[index] > s)
		{
			index--;
		}
		assert( index >= 0 );

		// Find where it lies between DAC level and next higher level, 
		// where 0.0 = DAC level, 0.5 = middle, 1.0 = next higher level
		double range = dac_table[index + 1] - dac_table[index];
		double fract = (s - dac_table[index]) / range;

		// Use fraction to determine whether to round up to next DAC level 
		if (full_dither_enabled)
		{
			// For dither case, use fraction as probability that next higher DAC level is used
			if (rand() > RAND_MAX * fract)
			{
				++index;
			}
		}
		else if (fract > 0.5) // round to closest 
		{
			++index;
		}

		out[i] = 14 - index;
	}
}


// *** Output conversion ***

static void psg_to_raw(std::vector<uint8_t>& in, const std::string& filename)
{
	printf("Writing to %s.psg.bin...", filename.c_str());
	std::ofstream f;
	f.exceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);
	f.open(filename, std::ifstream::binary);

	for (std::size_t i = 0; i < in.size(); i += 2)
	{
		f.put(in[i] << 4 | in[i + 1]);
	}
	printf("done\n");
}

static void psg_to_pcm(const std::vector<uint8_t>& in, std::vector<int16_t>& out)
{
	for (std::size_t i = 0; i < in.size(); ++i)
	{
		int index = 14 - in[i];
		out[i] = (int16_t)(dac_table[index] * 0x7FFF);
	}
}

enum
{
	cmd_gg_stereo = 0x4F,
	cmd_psg = 0x50,
	cmd_eof = 0x66,
	cmd_short_delay = 0x70
};

// Outputs PSG data to VGM file 
static void psg_to_vgm(const std::vector<uint8_t>& in, const std::string& filename, int sample_repeat)
{
	std::ofstream f;
	f.exceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);
	f.open(filename, std::ifstream::binary);

	char header[0x40] = "Vgm " "    " "\0\1\0\0";
	uint32_t file_size = in.size() * 3 + 0x40 - 4;
	uint32_t psg_clock = 3579545;
	header[4] = (file_size >> 0) & 0xff;
	header[5] = (file_size >> 8) & 0xff;
	header[6] = (file_size >> 16) & 0xff;
	header[7] = (file_size >> 24) & 0xff;
	header[13] = (psg_clock >> 0) & 0xff;
	header[14] = (psg_clock >> 8) & 0xff;
	header[15] = (psg_clock >> 16) & 0xff;
	header[16] = (psg_clock >> 24) & 0xff;
	f.write(header, 0x40)
	 .put(cmd_gg_stereo).put((char)0xFF)
	 .put(cmd_psg).put((char)0x80)
	 .put(cmd_psg).put((char)0x00);

	for (auto psg_volume: in)
	{
		f.put(cmd_psg).put(0x90 + psg_volume)
		 .put(cmd_short_delay + sample_repeat - 1);
	}

	f.put(cmd_eof);
}

class Args
{
	std::map<std::string, std::string> _args;

public:
	Args(int argc, char** argv)
	{
		bool haveLastKey = false;
		std::map<std::string, std::string>::iterator lastKey;
		for (int i = 1; i < argc; ++i)
		{
			switch (argv[i][0])
			{
			case '/':
			case '-':
				// Store as a valueless key
				lastKey = _args.insert(make_pair(std::string(argv[i] + 1), "")).first;
				haveLastKey = true;
				// Remember it
				break;
			case '\0':
				break;
			default:
				// Must be a value for the last key, or a filename
				if (haveLastKey)
				{
					lastKey->second = argv[i];
				}
				else
				{
					_args.insert(std::make_pair("filename", argv[i]));
				}
				// Clear it so we don't put the filename in the wrong place
				haveLastKey = false;
				break;
			}
		}
	}

	std::string getString(const std::string& name, const std::string& defaultValue)
	{
		std::map<std::string, std::string>::const_iterator it = _args.find(name);
		if (it == _args.end())
		{
			return defaultValue;
		}
		return it->second;
	}

	int getInt(const std::string& name, int defaultValue)
	{
		std::map<std::string, std::string>::const_iterator it = _args.find(name);
		if (it == _args.end())
		{
			return defaultValue;
		}
		return atoi(it->second.c_str());
	}

	double getDouble(const char* name, double defaultValue)
	{
		std::map<std::string, std::string>::const_iterator it = _args.find(name);
		if (it == _args.end())
		{
			return defaultValue;
		}
		return atof(it->second.c_str());
	}

	bool getBool(const std::string& name, bool defaultValue)
	{
		std::map<std::string, std::string>::const_iterator it = _args.find(name);
		if (it == _args.end())
		{
			return defaultValue;
		}
		const std::string& value = it->second;
		if (value.empty())
		{
			// Arg exists, but has no value, so we invert the default
			return !defaultValue;
		}
		switch (::tolower(value[0]))
		{
		case '0':
		case 'n':
		case 'f':
			return false;
		}
		return true;
	}

	bool exists(const std::string& name) const
	{
		return _args.find(name) != _args.end();
	}
};

int main(int argc, char** argv)
{
	try
	{
		// Parse args
		Args args(argc, argv);
		// Controls the low-pass effect used when making the waveform positive. Higher sampling rates
		// need higher values to achieve the same frequency cutoff, e.g. 10 works well at 44kHz.
		// Use save_offset to see the smoothness of the offset and hear its frequencies.
		int high_pass_shift = args.getInt("high_pass_shift", 6);
		// Set to true to enable full dithering, which is noisy 
		bool full_dither_enabled = args.getBool("full_dither_enabled", false);
		// Adjusts mild dither added all the time. 
		double mild_dither_level = args.getDouble("mild_dither_level", 0.01);
		// Affects output rate of VGM file. Effective output rate is 44100 Hz / sample_repeat.
		// For example, a value of 4 results in a sample rate of 11025 Hz.
		int sample_repeat = args.getInt("sample_repeat", 4);
		// Set to true to write out the "positive-ised" wave
		bool save_positive = args.getBool("save_positive", false);
		// Set to 1 to record the offset to keep the waveform positive.
		// Useful to understand how this part of the algorithm works.
		bool save_offset = args.getBool("save_offset", false);
		// Non-linearity of DAC 
		double const db_per_dac_step = args.getDouble("db_per_dac_step", 2.0);

		const std::string& filename = args.getString("filename", "in.wav");

		// Read input wave file, which must be 16-bit mono 
		printf("Reading %s...", filename.c_str());
		FileReader f(filename);

		// Just assume that wave data is at offset 0x2c, which it is for a "clean" WAV
		int const offset = 0x2c;
		std::vector<uint8_t> header;
		f.read(header, offset);
		const int numSamples = (f.size() - offset) / 2;
		std::vector<int16_t> samples;
		samples.reserve(numSamples);
		f.read(samples, numSamples);
		printf("done\n");

		printf("Preprocessing to positive with decay strength %d...", high_pass_shift);
		std::vector<int16_t> offsets;
		offsets.reserve(samples.size());
		make_positive(samples, offsets, high_pass_shift);
		printf("done\n");

		if (save_offset)
		{
			write_file(header, offsets, filename + ".offset.wav");
		}

		if (save_positive)
		{
			write_file(header, samples, filename + ".positive.wav");
		}

		init_dac_table(db_per_dac_step);
		std::vector<uint8_t> psg_data;
		psg_data.reserve(samples.size());
		printf("Converting to PSG data with ");
		if (full_dither_enabled)
		{
			printf("full dithering...");
		}
		else
		{
			printf("mild dithering level %f...", mild_dither_level);
		}
		pcm_to_psg(samples, psg_data, mild_dither_level, full_dither_enabled);
		printf("done\n");

		psg_to_raw(psg_data, filename + ".psg.bin");

		psg_to_pcm(psg_data, samples);
		write_file(header, samples, filename + ".out.wav");

		printf("Writing to %s.out.vgm...", filename.c_str());
		psg_to_vgm(psg_data, filename + ".out.vgm", sample_repeat);
		printf("done\n");

		return 0;
	}
	catch (const std::exception& e)
	{
		printf("Error: %s", e.what());
		return 1;
	}
}
