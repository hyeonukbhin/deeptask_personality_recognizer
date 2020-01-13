#!/usr/bin/python2.7
# -*- coding: utf-8 -*-
# [START speech_transcribe_streaming_mic]
from __future__ import division

import re
import sys

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
# import google.cloud.s
from six.moves import queue
import rospkg
import os
import rospy
from std_msgs.msg import String
from audio_msgs.msg import AudioData
import threading
import json
reload(sys)
sys.setdefaultencoding('utf-8')

# [set path]
pack_path = rospkg.RosPack().get_path("speech_to_text_converter")
service_key_path = pack_path + "/scripts/service_key.json"

os.environ[
    "GOOGLE_APPLICATION_CREDENTIALS"] = service_key_path

# Audio recording parameters
CHANNELS = 1
RATE = 44100
LOOP_RATE = 5
CHUNK = int(RATE / LOOP_RATE)  # 100ms


class RosTopicStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self, rate, chunk):
        global pub
        self._rate = rate
        self._chunk = chunk
        self._buff = queue.Queue()
        rospy.init_node('KIST_stt_converter', anonymous=False)
        pub = rospy.Publisher('recognitionResult', String, queue_size=100)
        rospy.Subscriber("audio_stream", AudioData, self.packet_callback)
        self.closed = True

    def __enter__(self):
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        # self._audio_stream.stop_stream()
        # self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        # self._audio_interface.terminate()

    # def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
    #     """Continuously collect data from the audio stream, into the buffer."""
    #     self._buff.put(in_data)
    #     return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]
            # print(data)
            # print(chunk)
            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                    # print(data)
                    # print(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)

    def packet_callback(self, packet):
        caller_speech = packet.data
        byte_str = self.make_bytestr(caller_speech)
        self._buff.put(byte_str)

    def make_bytestr(self, int16Array):
        byte_str = "".join(map(chr, int16Array))
        # byte_str = struct.pack('>%si' % len(int16Array), *int16Array)
        return byte_str


def listen_print_loop(responses):
    """Iterates through server responses and prints them.
    The responses passed is a generator that will block until a response
    is provided by the server.
    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.
    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """
    num_chars_printed = 0
    # print("d")
    test_idx = 0
    # print(responses)
    for response in responses:

        if not response.results:
            continue

        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # Display the transcription of the top alternative.
        transcript = result.alternatives[0].transcript


        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        #
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        overwrite_chars = ' ' * (num_chars_printed - len(transcript))
        # print(type(result))
        # print(result)
        import google.api_core.grpc_helpers
        # google.api_core.grpc_helpers._StreamingResponseIterator
        # print(response.results[0])
        if not result.is_final:
            # sys.stdout.write(transcript + overwrite_chars + '\r')
            # sys.stdout.flush()
            # print(test_idx)

            # print(transcript + overwrite_chars + '\r')
            # print(text_idx)

            test_idx +=1
            num_chars_printed = len(transcript)

        else:
            # print(transcript + overwrite_chars)
            result = str(transcript + overwrite_chars)

            send_topic(result)
            # print(result)
            C_END = "\033[0m"
            C_BOLD = "\033[1m"
            C_INVERSE = "\033[7m"

            C_BLACK = "\033[30m"
            C_RED = "\033[31m"
            C_GREEN = "\033[32m"
            C_YELLOW = "\033[33m"
            C_BLUE = "\033[34m"
            C_PURPLE = "\033[35m"
            C_CYAN = "\033[36m"
            C_WHITE = "\033[37m"
            # C_BOLD + C_WHITE + "가을 길은
            print("User Speech :" +C_BOLD + C_WHITE + result + C_END)
            # print(": {}".format(result))


            # Exit recognition if any of the transcribed phrases could be
            # one of our keywords.
            if re.search(r'\b(exit|quit)\b', transcript, re.I):
                print('Exiting..')
                break

            num_chars_printed = 0


def send_topic(sentence):
    current_time = rospy.get_rostime()
    input_string = str(sentence)
    name = ""
    jsonSTTFrame = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "perception",
            "target": ["planning", "dialog"],
            "content": ["human_speech"]
        },
        "human_speech": {
            "name": name,
            "speech": "%s" % input_string
        }
    }

    rospy.set_param("perception/human_speech", False)
    json_string = json.dumps(jsonSTTFrame, ensure_ascii=False, indent=4)
    pub.publish(json_string)


def sttConverter():
    language_code = 'ko-KR'  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        # single_utterance=True,
        sample_rate_hertz=RATE,
        language_code=language_code)
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    with RosTopicStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        # responses = client.streaming_recognize(streaming_config, requests)
        # # Now, put the transcription responses to use.
        # listen_print_loop(responses)  # signal.pause()

        try:
            responses = client.streaming_recognize(streaming_config, requests)
        except Exception as exception:
            # Output unexpected Exceptions.
            print("exception exception exception exception exception ")

        # Now, put the transcription responses to use.
        try:
            listen_print_loop(responses)

            # signal.pu
        except Exception as exception:
            # Output unexpected Exceptions.
            # print("")
            return
            # listen_print_loop(responses)
    # print("tttt")
    sttConverter()
    rospy.spin()

if __name__ == '__main__':
    while True:
        try:
            sttConverter()
        except Exception as e:
            # print(e)
            print("")
            # print("...")
