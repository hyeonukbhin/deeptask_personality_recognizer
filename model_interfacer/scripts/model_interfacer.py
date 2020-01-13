#!/usr/bin/python3
# -*- coding: utf-8 -*-
import pandas as pd
import gensim
from sklearn.externals import joblib
import rospy
from std_msgs.msg import String
# from pprintpp import pprint
import json
import rospkg

PACKAGE_PATH = rospkg.RosPack().get_path("model_interfacer")
MODEL_FILEPATH = PACKAGE_PATH + "/scripts/models/"

def read_df(filename):
    df = pd.read_csv(filename, sep=',', na_values=".", index_col=0)
    return df

def map_list_type(l, dtype=str):
    return list(map(dtype, l))

def get_name(label):
    if label == 0:
        name = "\x1b[1;31m{}\x1b[1;m".format(" ||     ")
    elif label == 1:
        name = "\x1b[1;32m{}\x1b[1;m".format(" ||||   ")
    elif label == 2:
        name = "\x1b[1;34m{}\x1b[1;m".format(" |||||| ")
    else:
        name = ""

    return name

def get_name_lmh(label):
    if label == 0:
        name = "low"
    elif label == 1:
        name = "middle"
    elif label == 2:
        name = "high"
    else:
        name = ""

    return name

def print_tui(lr_result):

    print("================================================")
    print("")
    print("\x1b[1;33m{}\x1b[1;m".format("Doc2Vec Features + Gradient Boosting Classifier"))
    print("")
    print("")
    print("")
    print("{} {} {} {} {}".format(get_name(lr_result[0]),get_name(lr_result[1]),get_name(lr_result[2]),get_name(lr_result[3]),get_name(lr_result[4])))
    print("{} {} {} {} {}".format(get_name(lr_result[0]),get_name(lr_result[1]),get_name(lr_result[2]),get_name(lr_result[3]),get_name(lr_result[4])))
    print("[ EXT. ] [ NEU. ] [ AGR. ] [ CON. ] [ OPN. ]")
    print("[외향성] [신경성] [친화성] [성실성] [개방성]")
    print("")
    print("================================================")

def personality_model(tokens):
    # 모델 가져오기
    model_doc2vec = gensim.models.Doc2Vec.load(MODEL_FILEPATH+"personality_doc2vec.model")

    test_x = model_doc2vec.infer_vector(tokens)

    model_lr_classfier_ext = joblib.load(MODEL_FILEPATH+"lr_ext.sav")
    model_lr_classfier_neu = joblib.load(MODEL_FILEPATH+"lr_neu.sav")
    model_lr_classfier_agr = joblib.load(MODEL_FILEPATH+"lr_agr.sav")
    model_lr_classfier_con = joblib.load(MODEL_FILEPATH+"lr_con.sav")
    model_lr_classfier_opn = joblib.load(MODEL_FILEPATH+"lr_opn.sav")

    lr_result = [model_lr_classfier_ext.predict([test_x])[0],
                 model_lr_classfier_neu.predict([test_x])[0],
                 model_lr_classfier_agr.predict([test_x])[0],
                 model_lr_classfier_con.predict([test_x])[0],
                 model_lr_classfier_opn.predict([test_x])[0]
                 ]

    model_gb_classfier_ext = joblib.load(MODEL_FILEPATH+"gb_ext.sav")
    model_gb_classfier_neu = joblib.load(MODEL_FILEPATH+"gb_neu.sav")
    model_gb_classfier_agr = joblib.load(MODEL_FILEPATH+"gb_agr.sav")
    model_gb_classfier_con = joblib.load(MODEL_FILEPATH+"gb_con.sav")
    model_gb_classfier_opn = joblib.load(MODEL_FILEPATH+"gb_opn.sav")

    gb_result = [model_gb_classfier_ext.predict([test_x])[0],
                 model_gb_classfier_neu.predict([test_x])[0],
                 model_gb_classfier_agr.predict([test_x])[0],
                 model_gb_classfier_con.predict([test_x])[0],
                 model_gb_classfier_opn.predict([test_x])[0]
                 ]

    result = lr_result
    return result

def send_recognition(name, result):
    current_time = rospy.get_rostime()
    msgs_dict = {
        "header": {
            "timestamp": "%i.%i" % (current_time.secs, current_time.nsecs),
            "source": "perception",
            "target": ["planning", "action"],
            "content": ["human_personality"]
        },
        "human_personality": {
            "name": name,
            "extraversion": get_name_lmh(result[0]),
            "neuroticism": get_name_lmh(result[1]),
            "agreeableness": get_name_lmh(result[2]),
            "Conscientiousness": get_name_lmh(result[3]),
            "openness": get_name_lmh(result[4])
        }
    }
    json_string = json.dumps(msgs_dict, ensure_ascii=False, indent=4)
    pub_recognition.publish(json_string)


def get_header(json_dict):
    source = json_dict["header"]["source"]
    target_list = json_dict['header']["target"]
    content_list = json_dict['header']["content"]

    return source, target_list, content_list

def callback_document(data):
    # json_dict = json.loads(data.data.decode('utf-8'))
    json_dict = json.loads(data.data)
    source, target_list, content_list = get_header(json_dict)
    # 사람 위치 추적 및 이름 파라미터 업데이
    if ("perception" in target_list) and (source == "perception") and ("document_result" in content_list):

        name = json_dict["document_result"]["name"]
        tokens = json_dict["document_result"]["tokens"]

        score = personality_model(tokens)

        if len(tokens) < 5:
            print_tui([4, 4, 4, 4, 4])

        else:
            print_tui(score)

        send_recognition(name, score)


def model_interface():
    global pub_recognition
    rospy.init_node('KIST_model_interface', anonymous=False)
    rospy.Subscriber("documentResult", String, callback_document)
    pub_recognition = rospy.Publisher("recognitionResult", String, queue_size=100)

    rospy.spin()

if __name__ == '__main__':
    model_interface()
