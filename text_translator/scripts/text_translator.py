#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
# from pprintpp import pprint
import json
from googletrans import Translator


def send_translation(name, speech_kr, speech_en):
    current_time = rospy.get_rostime()
    msgs_dict = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "perception",
            "target": ["perception"],
            "content": ["translation_result"]
        },
        "translation_result": {
            "name": name,
            "speech_kr": speech_kr,
            "speech_en": speech_en
        }
    }
    json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
    pub_translation.publish(json_string)


def get_header(json_dict):
    source = json_dict["header"]["source"]
    target_list = json_dict['header']["target"]
    content_list = json_dict['header']["content"]

    return source, target_list, content_list

def callback_speech(data):
    json_dict = json.loads(data.data)
    source, target_list, content_list = get_header(json_dict)
    # 사람 위치 추적 및 이름 파라미터 업데이
    if ("dialog" in target_list) and (source == "perception") and ("human_speech" in content_list):

        name = json_dict["human_speech"]["name"]
        speech_kr = json_dict["human_speech"]["speech"]

        translator = Translator()

        speech_en = translator.translate(speech_kr).text

        print(speech_kr)
        print(speech_en)
        print("")
        send_translation(name, speech_kr, speech_en)


def text_translator():
    global pub_translation
    rospy.init_node('KIST_text_translator', anonymous=False)
    rospy.Subscriber("recognitionResult", String, callback_speech)
    pub_translation = rospy.Publisher("translationResult", String, queue_size=100)
    rospy.spin()

if __name__ == '__main__':
    text_translator()

