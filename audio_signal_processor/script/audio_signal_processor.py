#!/usr/bin/python3.5
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
import parselmouth
import rospy
from audio_msgs.msg import AudioData, FeatureData
import sounddevice as sd

time_step = 0.05
pitch_floor = 75.0
max_number_of_candidates = 15
very_accurate = True
silence_threshold = 0.03
voicing_threshold = 0.45
octave_cost = 0.01
octave_jump_cost = 0.35
voiced_unvoiced_cost = 0.14
ceiling = 600.0

LOOP_RATE = 10


# if rospy.has_param('~DEVICE_INDEX'):
#     INPUT_DEVICE = rospy.get_param('~DEVICE_INDEX')
#     CHANNELS = rospy.get_param('~CHANNELS')
#     RATE = rospy.get_param('~RATE')
#     LOOP_RATE = rospy.get_param('~LOOP_RATE')
#     CHUNK = rospy.get_param('~CHUNK')
#     # CHUNK = int(RATE / LOOP_RATE)  # 100ms
#
# else:

class AudioSignalProcessor:
    # Calibration pair distance parameter
    def __init__(self):
        rospy.init_node("audio_signal_processor", anonymous=False)
        rospy.Subscriber("audio_stream", AudioData, self.callback_packet, queue_size=50)
        self.pub_voice_feature = rospy.Publisher("voice_feature", FeatureData, queue_size=50)
        self.wav_array = np.array([])
        self.voice_feature = FeatureData()
        rospy.spin()

    def callback_packet(self, topic):
        audio_stream = topic.data
        msg_sequence = topic.header.seq
        byte_str = self.make_byte_str(audio_stream)
        wav_raw = self.make_wav_raw(byte_str)
        self.wav_array = np.append(self.wav_array, wav_raw)

        if msg_sequence > 0 and msg_sequence % LOOP_RATE == 0:

            amplitude = self.amp_calculation(self.wav_array)
            pitch = self.pitch_calculation(self.wav_array)
            intensity = self.intensity_calculation(self.wav_array)

            self.voice_feature.header.seq = msg_sequence
            self.voice_feature.header.frame_id = "voice_feature"
            self.voice_feature.header.stamp = rospy.Time.now()
            self.voice_feature.amplitude = amplitude[:30]
            self.voice_feature.pitch = pitch
            self.voice_feature.intensity = intensity
            human_speech = rospy.get_param("perception/human_speech")

            if human_speech is True:
                self.pub_voice_feature.publish(self.voice_feature)
                # self.play_audio(amplitude)
            self.wav_array = np.array([])


    def make_byte_str(self, int16Array):
        byte_str = "".join(map(chr, int16Array))
        return byte_str

    def make_wav_raw(self, byte_str):
        wav_raw = np.fromstring(byte_str, dtype=np.int16).astype(np.float64) / 32768
        return wav_raw

    def amp_calculation(self, wav_array):
        snd = parselmouth.Sound(wav_array)
        return snd.values.T

    def pitch_calculation(self, wav_array):
        snd = parselmouth.Sound(wav_array)
        pitch = snd.to_pitch_ac(time_step=time_step,
                                pitch_floor=pitch_floor,
                                max_number_of_candidates=max_number_of_candidates,
                                very_accurate=very_accurate,
                                silence_threshold=silence_threshold,
                                voicing_threshold=voicing_threshold,
                                octave_cost=octave_cost,
                                octave_jump_cost=octave_jump_cost,
                                voiced_unvoiced_cost=voiced_unvoiced_cost,
                                pitch_ceiling=ceiling
                                )
        pitch_values = np.array(pitch.selected_array['frequency'])
        pitch_xs = np.round(np.array(pitch.xs()), 4)

        return pitch_values

    def intensity_calculation(self, wav_array):
        snd = parselmouth.Sound(wav_array)
        intensity = snd.to_intensity(minimum_pitch=pitch_floor,
                                     time_step=time_step,
                                     subtract_mean=True)

        intensity_values = np.array(intensity.values.T)
        intensity_xs = np.round(np.array(intensity.xs()), 4)

        return intensity_values

    def interval_calculation(self, wav_array):
        snd = parselmouth.Sound(wav_array)
        return ""

    def play_audio(self, myarray):
        sd.default.samplerate = 44100
        # sd.default.device =
        sd.play(myarray)



if __name__ == '__main__':
    try:
        AudioSignalProcessor()
    except rospy.ROSInterruptException:
        pass
