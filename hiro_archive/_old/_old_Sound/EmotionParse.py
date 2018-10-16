import numpy as np
import wave
from scipy.io import wavfile
import pyaudio

def speedx(sound_array, factor):
    """ Multiplies the sound's speed by some `factor` """
    indices = np.round( np.arange(0, len(sound_array), factor) ) #creates a list of indices in the
    #Sound array to either skip or duplicate.
    indices = indices[indices < len(sound_array)].astype(int) #Takes those indices and then
    #cast all of them less than the length of the sound array as int.
    return sound_array[ indices.astype(int) ]

def stretch(sound_array, f, window_size=8192, h=2048):
    """ Stretches the sound by a factor `f` """

    phase  = np.zeros(window_size)
    hanning_window = np.hanning(window_size)
    result = np.zeros( len(sound_array) /f + window_size)

    for i in np.arange(0, len(sound_array)-(window_size+h), h*f):

        # two potentially overlapping subarrays
        a1 = sound_array[i: i + window_size]
        a2 = sound_array[i + h: i + window_size + h]

        # resynchronize the second array on the first
        s1 =  np.fft.fft(hanning_window * a1)
        s2 =  np.fft.fft(hanning_window * a2)
        phase = (phase + np.angle(s2/s1)) % 2*np.pi
        a2_rephased = np.fft.ifft(np.abs(s2)*np.exp(1j*phase))

        # add to result
        i2 = int(i/f)
        result[i2 : i2 + window_size] += hanning_window*a2_rephased

    result = ((2**(16-4)) * result/result.max()) # normalize (16bit)

    return result.astype('int16')

def pitchshift(sound_array, n, window_size=2**13, h=2**11):
    """ Changes the pitch of a sound by ``n`` semitones. """
    factor = 2**(1.0 * n / 12.0)
    stretched = stretch(snd_array, 1.0/factor, window_size, h)
    return speedx(stretched[window_size:], factor)


def volumeshift(sound_array, factor):
	#Increases or decreases the volume of the array.
	return np.multiply(sound_array,factor)

def load(sound_file):
	w = wave.open(sound_file, "rb")

	return w

def play(sound):
	#define stream chunk
	chunk = 1024
	p = pyaudio.PyAudio()

	params = sound.getparams()
	f = params[3] # number of frames

	#open stream
	stream = p.open(format = p.get_format_from_width(params[1]),
                channels = params[0],
                rate = params[2],
                output = True)

	data = sound.readframes(chunk)

	#Play stream
	while data != '':
		stream.write(data)
		data = sound.readframes(chunk)
	sound.close()

def play2(filename):
    fs, data = wavfile.read(filename)
    spd2 = speedx(data, 4)

    # scaled = np.int16(data/np.max(np.abs(data)) * 32767)
    wavfile.write('test.wav', len(data), data)
    wavfile.write('spd2.wav', len(spd2), spd2)

if __name__ == '__main__':
    fn = "./media/sad.wav"
    play2(fn)
