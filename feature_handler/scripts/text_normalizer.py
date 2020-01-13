#!/usr/bin/python3
# -*- coding: utf-8 -*-
import pandas as pd
import re
import nltk
import rospy
from std_msgs.msg import String
import json
import rospkg
import csv

PACKAGE_PATH = rospkg.RosPack().get_path("feature_handler") + "/scripts/"

def read_df(filename):
    df = pd.read_csv(filename, sep=',', na_values=".", index_col=0, encoding="utf-8")
    return df


def drop_under_N_sentence(df, authid_list, drop_n):
    for authid in authid_list:
        df_id = df[df['#AUTHID'] == authid]
        count = len(df_id.index)
        if count < drop_n:
            df = df[df['#AUTHID'] != authid]
    return df


def cleanText(readData):
    # 텍스트에 포함되어 있는 특수 문자 제거
    text = re.sub('[-=+,#/\?:^$.@*\"※~&%ㆍ!』\\‘|\(\)\[\]\<\>`\'…》]', '', readData)
    return text


def map_list_type(l, dtype=str):
    return list(map(dtype, l))


def tokenize_one_doc(df):
    status_list = df["speech_en"].tolist()
    cleaned_tokens_2d_list = [nltk.word_tokenize(cleanText(sentence)) for sentence in status_list]
    splited_token_list = [t for d in cleaned_tokens_2d_list for t in d]
    import spacy
    nlp = spacy.load('en')
    splited_token_list = [term for term in splited_token_list if term not in nlp.Defaults.stop_words]
    return splited_token_list


def save_df(df, filename):
    df.to_csv(filename, mode="w", sep=',')


def send_document(name, tokens):
    current_time = rospy.get_rostime()
    msgs_dict = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "perception",
            "target": ["perception"],
            "content": ["document_result"]
        },
        "document_result": {
            "name": name,
            "tokens": tokens
        }
    }


    json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
    pub_document.publish(json_string)

    print(tokens)


def get_header(json_dict):
    source = json_dict["header"]["source"]
    target_list = json_dict['header']["target"]
    content_list = json_dict['header']["content"]

    return source, target_list, content_list


def callback_translation(data):
    # json_dict = json.loads(data.data.decode('utf-8'))
    json_dict = json.loads(data.data)
    source, target_list, content_list = get_header(json_dict)
    if ("perception" in target_list) and (source == "perception") and ("translation_result" in content_list):
        name = json_dict["translation_result"]["name"]
        speech_kr = json_dict["translation_result"]["speech_kr"]
        speech_en = json_dict["translation_result"]["speech_en"]

        if speech_en != "" or None:

            try:
                data = read_df(PACKAGE_PATH + "data.csv")
            except Exception as exception:

                f = open(PACKAGE_PATH + "data.csv", 'w', encoding='utf-8')
                wr = csv.writer(f)
                wr.writerow(["", "name", "speech_en", "speech_kr"])
                f.close()
                data = read_df(PACKAGE_PATH + "data.csv")

            data_row = pd.Series([name, speech_kr, speech_en], index=["name", "speech_kr", "speech_en"])
            data = data.append(data_row, ignore_index=True)
            data = data[-20:].reset_index(drop=True)
            save_df(data, PACKAGE_PATH + "data.csv")

            tokens = tokenize_one_doc(data)
            send_document(name, tokens)


def text_normalizer():
    global pub_document
    rospy.init_node('KIST_text_normalizer', anonymous=False)
    rospy.Subscriber("translationResult", String, callback_translation)
    pub_document = rospy.Publisher("documentResult", String, queue_size=100)

    rospy.spin()


if __name__ == '__main__':
    text_normalizer()
