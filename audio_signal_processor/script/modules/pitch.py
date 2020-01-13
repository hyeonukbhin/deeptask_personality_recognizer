#!/usr/bin/python3.5
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd
import parselmouth
from scipy.interpolate import lagrange, interp1d
from scipy import stats


def get_pitch(filepath, **kwargs):
    df_pitch = None
    snd = None
    time_step = 0.01,
    pitch_floor = 75.0,
    max_number_of_candidates = 15,
    very_accurate = True,
    silence_threshold = 0.03,
    voicing_threshold = 0.45,
    octave_cost = 0.01,
    octave_jump_cost = 0.35,
    voiced_unvoiced_cost = 0.14,
    ceiling = 600.0
    for key, value in kwargs.items():
        if key is "time_step": time_step = value
        if key is "pitch_floor": pitch_floor = value
        if key is "max_number_of_candidates": max_number_of_candidates = value
        if key is "very_accurate": very_accurate = value
        if key is "silence_threshold": silence_threshold = value
        if key is "voicing_threshold": voicing_threshold = value
        if key is "octave_cost": octave_cost = value
        if key is "octave_jump_cost": octave_jump_cost = value
        if key is "voiced_unvoiced_cost": voiced_unvoiced_cost = value
        if key is "ceiling": ceiling = value
    if ".txt" in filepath:
        df = pd.read_csv(filepath, sep="\s+")
        df_pitch = df.replace("--undefined--", 0).astype("float64")
        # snd = ""
    if ".wav" in filepath:
        snd = parselmouth.Sound(filepath)
        pitch = snd.to_pitch_ac(time_step=time_step,
                                pitch_floor=pitch_floor,
                                max_number_of_candidates=max_number_of_candidates,
                                very_accurate=very_accurate,
                                silence_threshold=silence_threshold,
                                voicing_threshold=voicing_threshold,
                                octave_cost=octave_cost,
                                octave_jump_cost=octave_jump_cost,
                                voiced_unvoiced_cost=voiced_unvoiced_cost,
                                ceiling=ceiling
                                )
        pitch_values = np.array(pitch.selected_array['frequency'])
        pitch_xs = np.round(np.array(pitch.xs()), 4)
        pitch_1 = np.c_[pitch_xs, pitch_values]
        df_pitch = pd.DataFrame(pitch_1, columns=["Time_s", "F0_Hz"])

    return df_pitch, snd


def compare(value, df_vad):
    result = "OFF"
    for row in df_vad.itertuples():
        if (value >= row.ON) and (value <= row.OFF):
            # print(row.Index)
            result = "ON"
            index = row.Index
    return result


def get_vad_index(df_pitch, time_step=0.1, off_threshold=0.5, on_threshold=0.5):
    count = 0
    # print(threshold)
    buff = False
    for row in df_pitch.itertuples():
        if row.VAD == "ON":
            #온이 하나라도 있으면
            check_on_length = df_pitch.loc[row.Index - int(off_threshold / time_step):row.Index - 1, "VAD"].isin(["ON"]).tolist()
            #오프가 하나라도 있으면
            check_off_lenth = df_pitch.loc[row.Index: row.Index + int(on_threshold / time_step) - 1, "VAD"].isin(["OFF"]).tolist()
            if buff is False and True not in check_on_length and True not in check_off_lenth:
                count += 1
                buff = True
            df_pitch.loc[row.Index, "VAD_INDEX"] = count
        else:
            df_pitch.loc[row.Index, "VAD_INDEX"] = 0
            buff = False
    return df_pitch


def add_vad(df_pitch, df_vad=None, time_step=0.1, off_threshold=0.5, on_threshold=0.5, **kwargs):
    df_return = None
    for key, value in kwargs.items():
        if key is "source" and value is "praat":
            df_pitch["VAD"] = np.where(df_pitch["F0_Hz"] != 0, "ON", "OFF")
            df_return = get_vad_index(df_pitch, time_step=time_step, off_threshold=off_threshold, on_threshold=on_threshold)
        if key is "source" and value is "manual":
            df_pitch["VAD"] = df_pitch["Time_s"].map(lambda x: compare(x, df_vad))
            df_return = get_vad_index(df_pitch, time_step=time_step, off_threshold=off_threshold, on_threshold=on_threshold)

    return df_return


def make_delta_pitch(df):
    df = df.reset_index(drop=True)
    remove_idx = []
    df["DELTA_PITCH"] = df["RAW_PITCH"] - df['RAW_PITCH'].shift(1)
    for row in df.itertuples():
        if (row.Index > 0) and (df.loc[row.Index - 1, "VAD"] == "OFF") and (df.loc[row.Index, "VAD"] == "ON"):
            remove_idx.append(row.Index)
    df_return = df.drop(df.index[remove_idx])
    df_return = df_return.reset_index(drop=True)

    return df_return


def make_smooth_pitch(df, window=5, reject_vad=True):
    # 한 서브젝트의 Active Speech 당 지난 5개의 샘플을 에 대한 평균값

    if reject_vad is True:
        df = df[(df['VAD'] != "OFF")]
    # df = df.copy()
    df_return = pd.DataFrame(dtype=float)

    subject_list = get_index_list(df, collection="SUBJECT_NUM")
    for subject in subject_list:
        df_subject = df[(df['SUBJECT_NUM'] == subject)].copy()
        episode_list = get_index_list(df_subject, collection="EPISODE_NUM")
        for episode in episode_list:
            df_episode = df_subject[(df_subject['EPISODE_NUM'] == episode)].copy()
            vad_index_list = get_index_list(df_episode, collection="VAD_INDEX")
            for vad_index in vad_index_list:
                df_vad_index = df_episode[(df_episode['VAD_INDEX'] == vad_index)].copy()
                df_vad_index["SMOOTH_PITCH"] = df_vad_index["RAW_PITCH"].rolling(window=window, min_periods=1).mean()
                df_return = df_return.append(df_vad_index, ignore_index=True)

    df_return = df_return.reset_index(drop=True)

    return df_return


def get_index_list(df, collection="SUBJECT_NUM"):
    df_return = df[collection].drop_duplicates().values
    return df_return


def make_interpolation_function(df, func="linear", reject_vad=True):
    # interpolation option
    # 보간법을 썼을때는 resampling을 해야하는데....
    # 어떤식으로 의미가 있을가?
    # 일단 패스
    # 각 Speech Activityㄷ 당 해야함
    if reject_vad is True:
        df = df[(df['VAD'] != "OFF")]

    xs = df["Time_s"].values
    ys = df["RAW_PITCH"].values

    if func is "linear":
        f = interp1d(xs, ys, kind="linear")

    elif func is "quadratic":
        f = interp1d(xs, ys, kind="quadratic")

    elif func is "cubic":
        f = interp1d(xs, ys, kind="cubic")

    elif func is "lagrange":
        f = lagrange(xs, ys)

    return xs, ys, f


from fa_tools import df_handler

def make_vad_feature(df, reject_vad=True, collection="RAW_PITCH"):
    if reject_vad is True:
        df = df[(df['VAD'] != "OFF")]

    df_return = pd.DataFrame(dtype=float)
    df_return_plot = pd.DataFrame(dtype=float)

    subject_list = get_index_list(df, collection="SUBJECT_NUM")
    for subject in subject_list:
        df_subject = df[(df['SUBJECT_NUM'] == subject)].copy()
        episode_list = get_index_list(df_subject, collection="EPISODE_NUM")
        for episode in episode_list:
            df_episode = df_subject[(df_subject['EPISODE_NUM'] == episode)].copy()
            vad_index_list = get_index_list(df_episode, collection="VAD_INDEX")
            for vad_index in vad_index_list:
                df_vad_index = df_episode[(df_episode['VAD_INDEX'] == vad_index)].copy()

                ys = df_vad_index[collection]
                mean = ys.mean()
                median = ys.median()
                minimum = ys.min()
                maximum = ys.max()
                std = ys.std()
                trim_mean = stats.trim_mean(ys, 0.3)

                # print(df_vad_index["E"].iloc[0])
                # print(df_vad_index["N"].iloc[0])
                E = df_vad_index["E"].iloc[0]
                N = df_vad_index["N"].iloc[0]

                df_vad_index["MEAN"] = mean
                df_vad_index["MEDIAN"] = median
                df_vad_index["MINIMUM"] = minimum
                df_vad_index["MAXIMUM"] = maximum
                df_vad_index["STD"] = std
                df_vad_index["TRIM_MEAN"] = trim_mean

                s = pd.Series([subject, episode, vad_index, mean, median, minimum, maximum, std, trim_mean, E, N],
                              index=['SUBJECT_NUM', 'EPISODE_NUM', 'VAD_INDEX', "MEAN", "MEDIAN", "MINIMUM", "MAXIMUM", "STD", "TRIM_MEAN", "E", "N"])
                df_return = df_return.append(s, ignore_index=True)

                df_return_plot = df_return_plot.append(df_vad_index, ignore_index=True)


    df_return = df_return.reset_index(drop=True)
    df_return_plot = df_return_plot.reset_index(drop=True)

    return df_return, df_return_plot
    # print(ys)
    # print(trim_mean)



# y_lin = f_lin(xs)
# y_quad = f_quad(xs)
# y_cubic = f_cubic(xs)
# y_lagrange = f_lagrange(xs)


def make_baseline(df, feature="intensity"):
    df = df[(df['VAD'] == "OFF")]
    print(df)