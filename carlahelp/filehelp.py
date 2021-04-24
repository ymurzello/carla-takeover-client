'''
Copyright 2020, Rocky Liang, All rights reserved
'''
import os
import json
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np


def make_file_name(n, prefix='data_', nlen=6, ext='.h5'):
    '''
    return file name in carla dataset convention
    '''
    num_str = str(n)
    residual = nlen - len(num_str)
    for _ in range(residual):
        num_str = '0' + num_str
    return prefix + num_str + ext

def make_file_list(a,b, path):
    '''
    creates list of full file paths for data
    '''
    name_list = []
    for n in range(a,b+1):
        #6790 is a corrupt file
        if n != 6790:
            name = path + make_file_name(n)
            name_list.append(name)
    return name_list

def save_as_json(path, data):
    with open(path, 'w') as j:
        json.dump(data, j)

def read_json(n, path, key):
    '''loads json as dict and reads key value'''
    jname = make_file_name(n, prefix='target_', nlen=6, ext='.json')
    with open(os.path.join(path, jname)) as j:
        data = json.load(j)
        return data[key]

def read_json_config(path):
    '''return entire json as dict'''
    with open(path) as j:
        data = json.load(j)
        return data

def show_series(X, data_ind=0):
    '''
    takes batch of series images and plots them
    data_ind is a batch index
    '''
    seq_length = X[0].shape[1]
    plt.figure(figsize=(16,10))
    for i, img in enumerate(X[0][data_ind]):
        plt.subplot(1,seq_length,i+1)
        if(str(type(img))=="<class 'torch.Tensor'>"):
            img = np.moveaxis(img.numpy(),0,2)
        plt.imshow(img)
        plt.axis('off')
    #plt.show()

def date_string(note=None):
    '''
    function for naming tensorboard logs
    returns a string in the format of:
    month_date_year_hour_minute

    if note is given, it will be appended to
    the back of the string
    '''
    now = datetime.now()

    time_list = []
    #add 0 to value if single digit
    month_val = str(now.month)
    if len(month_val) == 1:
        month_val = '0' + month_val
    time_list.append(month_val)

    day_val = str(now.day)
    if len(day_val) == 1:
        day_val = '0' + day_val
    time_list.append(day_val)
    time_list.append(str(now.year))

    hour_val = str(now.hour)
    if len(hour_val) == 1:
        hour_val = '0' + hour_val
    time_list.append(hour_val)

    minute_val = str(now.minute)
    if len(minute_val) == 1:
        minute_val = str(0) + minute_val
    time_list.append(minute_val)

    second_val = str(now.second)
    if len(second_val) == 1:
        minute_val = str(0) + minute_val
    time_list.append(second_val)

    #convert all to string
    time_list_str = list(map(str, time_list))
    timepath = ''
    for i in time_list_str[:-1]:
        timepath += i
        timepath += '_'
    timepath += time_list_str[-1]

    #add note if it exists
    if note is not None:
        timepath = timepath + '_' + str(note)

    return timepath

if __name__=="__main__":
    try:
        data = read_json_config("spawn_configs/test.json")
    except:
        data = read_json_config("../spawn_configs/test.json")
    print(data)
