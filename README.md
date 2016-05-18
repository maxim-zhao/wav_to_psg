# wav_to_psg
Converts WAV data to SN76489 attenuation data with a novel algorithm to minimise quantisation error

This is a clone and minor tweak to a program by [blargg](https://github.com/gblargg) which he [posted at SMS Power!](http://www.smspower.org/forums/9686#44103), without permission or licence. The program takes a WAV file and applies a low-frequency offset (pretty much dynamic DC offsetting) to make the "interesting" parts of the data coincide with the more densely-populated parts of the SN76489's possible output levels. It then quantises the data to those levels, to allow generation of sample data at higher perceived quality than a simple unadjusted quantisation.
