import numpy as np
from scipy.io import wavfile
import wave

class SoundManipulator:
	def __init__(self, filename):
		rospy.init_node('edwin_emotionParse', anonymous = True)
		rospy.Subscriber('/behavior_cmd', String, self.demo_parse_emotion, queue_size=10)
		self.sound_pub = rospy.Publisher('edwin_sound', String, queue_size=2)

		self.filename = filename

	def speedx(self, filename, factor):
		""" Multiplies the sound's speed by some `factor` """
		fs, data = wavfile.read(filename)
		sound_array = data

		indices = np.round( np.arange(0, len(sound_array), factor) ) #creates a list of indices in the
		#Sound array to either skip or duplicate.  
		indices = indices[indices < len(sound_array)].astype(int) #Takes those indices and then 
		#cast all of them less than the length of the sound array as int.  
		new_array = sound_array[ indices.astype(int) ]

		wavfile.write("alter", fs, new_array)

	def time_accel(self, filename, factor): #The Emiya Family Magic now as a code function! jk.
		CHANNELS = 1
		swidth = 2
		Change_RATE = factor

		spf = wave.open(filename, 'rb')
		RATE=spf.getframerate()
		signal = spf.readframes(-1)

		wf = wave.open("alter", 'wb')
		wf.setnchannels(CHANNELS)
		wf.setsampwidth(swidth)
		wf.setframerate(RATE*Change_RATE)
		wf.writeframes(signal)
		wf.close()

	def stretch(self, filename, factor, window_size=2048, h=512):
		""" Stretches the sound by a factor """
		fs, data = wavfile.read(filename)
		sound_array = data

		phase  = np.zeros(window_size)
		hanning_window = np.hanning(window_size)
		result = np.zeros( len(sound_array) /factor + window_size)

		for i in np.arange(0, len(sound_array)-(window_size+h), h*factor):

			# two potentially overlapping subarrays
			a1 = sound_array[i: i + window_size]
			a2 = sound_array[i + h: i + window_size + h]

			# resynchronize the second array on the first
			s1 =  np.fft.fft(hanning_window * a1)
			s2 =  np.fft.fft(hanning_window * a2)
			phase = (phase + np.angle(s2/s1)) % 2*np.pi
			a2_rephased = np.fft.ifft(np.abs(s2)*np.exp(1j*phase))

			# add to result
			i2 = int(i/factor)
			result[i2 : i2 + window_size] += hanning_window*a2_rephased

		result = ((2**(16-4)) * result/result.max()) # normalize (16bit)
		result = result.astype('int16')

		wavfile.write('pitchshift.wav', len(result), result) #write



	def pitchshift(self, n, filename, window_size=2048, h=512):
		""" Changes the pitch of a sound by ``n`` semitones. """
		factor = 2**(1.0 * n / 12.0)
		self.speedx(filename, factor) #Write it.

		self.stretch("alter", 1.0/factor, window_size, h)
		#self.volumeshift("alter", 1.0)

	def volumeshift(self, filename, factor):
		#Increases or decreases the volume of the array.
		factor = 10**(factor/20.0)
		fs, data = wavfile.read(filename)
		data = np.multiply(data, factor)
		print factor
		wavfile.write("final_form", fs, data)

	def load(self, sound_file):
		w = wave.open(sound_file, "rb")

	def play(self, filename):
		fs, data = wavfile.read(filename)
		#scaled = np.int16(data/np.max(np.abs(data)) * 32767)

		wavfile.write('test.wav', fs, data)


	def parse_emotion(self, emotion_state):
		#Emotionstate being a list containing 4 emotional values
		#happy = 0, sad = 1, angry = 2, scared = 3

		total_state = sum(emotion_state) # for weighting
		happy = (emotion_state[0]/total_state) 
		sad = emotion_state[1]/total_state
		angry = emotion_state[2]/total_state
		scared = emotion_state[3]/total_state

		#happy: +pitch, - length, +.5 volume
		#sad: - pitch, + length, - volume
		#angry: -pitch, + length, + volume
		#scared +pitch, - length, - volume
		transform_pitch = 0
		transform_length = 0
		transform_volume = 0
		transform_factor = [transform_pitch, transform_length, transform_volume]
		transform_factor[0] = (happy-sad-angry+scared)*1.0 #arbitrary factors
		transform_factor[1] = 2**(1.0*(-happy+sad+angry-scared)/12)
		transform_factor[2] = (+.5*happy-sad+angry-scared)*1.0
		print transform_factor
		self.pitchshift(transform_factor[0], self.filename)
		self.stretch("pitchshift.wav", transform_factor[1])
		self.volumeshift("pitchshift.wav", transform_factor[2])
	

	def demo_parse_emotion(self, data):
		sound_request = data.data
		if sound_request == "greet":
			self.sound_pub.publish("surprise")
		elif sound_request == "sad":
			self.sound_pub.publish("sad")
		elif sound_request == "curiosity"
			self.sound_pub.publish("look")
		elif sound_request == "idle_1_lookaround"
			self.sound_pub.publish("unsure")


	def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
	# sound = load("r2d2.wav")
	#testfile = wave.open("./media/sad.wav", 'r')
	#print testfile.getnchannels()
	sound = SoundManipulator("./media/sad.wav")
	#sound.time_accel(sound.filename, .5)
	#emotion_state_test = [50., 30., 20., 0.] #0-100
	#sound.parse_emotion(emotion_state_test)
	sound.run()
