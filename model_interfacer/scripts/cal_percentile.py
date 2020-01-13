#-*- coding: utf-8 -*-
import pandas as pd
from collections import OrderedDict
from collections import namedtuple
from pprintpp import pprint

import gensim
import ast
from sklearn import preprocessing
from sklearn import utils
from sklearn.linear_model import LogisticRegression
import matplotlib as mpl
import matplotlib.pylab as plt
import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from sklearn.linear_model import LinearRegression, Ridge, Lasso, ElasticNet
from sklearn.pipeline import Pipeline
from sklearn.model_selection import KFold, cross_val_score, ShuffleSplit

from sklearn.feature_extraction.text import CountVectorizer, TfidfVectorizer
from sklearn.linear_model import LinearRegression, SGDRegressor, Lasso, Ridge
from sklearn.ensemble import GradientBoostingRegressor, ExtraTreesRegressor, RandomForestRegressor
import xgboost as xgb
from xgboost import XGBRegressor
from sklearn.pipeline import Pipeline
from sklearn.model_selection import KFold, cross_val_score, ShuffleSplit
from sklearn.metrics import mean_squared_error, make_scorer
from collections import defaultdict
from tabulate import tabulate
import seaborn as sns



def read_df(filename):
    df = pd.read_csv(filename, sep=',', na_values=".", index_col=0)
    return df

def act_fuction(x):
    if x > 3:
        y = 1
    else:
        y = 0
    return y



def plot_sklearn(train_x, train_y, model):
    plt.scatter(train_x, train_y)
    xx = np.linspace(0, 1, 1000)
    plt.plot(xx, model.predict(xx[:, np.newaxis]))
    plt.show()


def cv_rmse(model, X, y, cv=5, scoring='neg_mean_squared_error'):
    """ Compute an overall RMSE across all folds of cross validation"""

    print(cross_val_score(model, X, y, cv=cv, scoring='neg_mean_squared_error'), -1)
    print(type(cross_val_score(model, X, y, cv=cv, scoring='neg_mean_squared_error')))
    return np.mean(cross_val_score(model, X, y, cv=cv, scoring='neg_mean_squared_error'))*-1
    # return np.sqrt(np.mean(np.multiply(cross_val_score(model, X, y, cv=cv, scoring='neg_mean_absolute_error'), -1)))


# neg_mean_absolute_error = neg_mean_absolute_error_scorer,
# neg_mean_squared_error = neg_mean_squared_error_scorer,
# neg_mean_squared_log_error = neg_mean_squared_log_error_scorer,

def cv_accuracy(name, model, X, y, cv=5):
    """ Compute an overall RMSE across all folds of cross validation"""

    # print(name)
    result = np.mean(cross_val_score(model, X, y, cv=cv))
    # if name is "gbr_d2v" or "gbr_d2v" or "gbr_d2v":
    #
    # if (name == "gradient boosting reg.") or (name == "random forest reg.") or (name == "Xboosting reg."):
    #     result = result*-1

    return result

def RMSE(y_true, y_pred):
    """ Root Mean Squared Error"""

    return np.sqrt(np.mean((y_true - y_pred) ** 2))


def RMSLE(y_true, y_pred):
    """ Root Mean Squared Logarithmic Error"""

    return np.sqrt(np.mean(((np.log(y_true + 1) - np.log(y_pred + 1)) ** 2)))

def benchmark(model, X, y, n):
    ss = ShuffleSplit(n_splits=5, test_size=1 - n, random_state=0)
    scores = []
    for train, test in ss.split(X, y):
        scores.append(RMSE(np.array(y[test]), model.fit(X[train], y[train]).predict(X[test])))

        # print("best score :{}".format(model.))
        # print("test sixze :{}".format(len(test)))
    return np.mean(scores)

def main():
    df_docs = read_df("../../feature_handler/scripts/both_docs.csv")
    sentence_docs = df_docs["tokens"].tolist()
    score_docs = df_docs["p_score"].tolist()
    rating_arr = [list(map(float, ast.literal_eval(i))) for i in score_docs]



    df_score = pd.DataFrame(rating_arr)
    df_score.columns = ['sEXT', 'sNEU', 'sAGR', 'sCON', 'sOPN']

    # df_both.plot.hist()
    df_score.hist(bins=20)
    ext_pt = np.percentile(df_score["sEXT"], [0, 33.3, 66.7, 100], interpolation="nearest")
    neu_pt = np.percentile(df_score["sNEU"], [0, 33.3, 66.7, 100], interpolation="nearest")
    agr_pt = np.percentile(df_score["sAGR"], [0, 33.3, 66.7, 100], interpolation="nearest")
    con_pt = np.percentile(df_score["sCON"], [0, 33.3, 66.7, 100], interpolation="nearest")
    opn_pt = np.percentile(df_score["sOPN"], [0, 33.3, 66.7, 100], interpolation="nearest")


    arr_class_pt = np.array([ext_pt, neu_pt, agr_pt, con_pt, opn_pt])
    arr_class_pt = np.transpose(arr_class_pt)

    df_class_pt = pd.DataFrame(arr_class_pt, columns=['sEXT', 'sNEU', 'sAGR', 'sCON', 'sOPN'], index=[0, 0.33, 0.67, 1])

    print(df_class_pt)

    filename = "3c_percentile.csv"
    df_class_pt.to_csv(filename, mode="w", sep=',')
    df_read = read_df(filename)

    print(df_read)

main()