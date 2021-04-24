import matplotlib.pyplot as plt
import argparse
import os

from carlahelp.filehelp import read_json

def plot_data(key, recording):
    length = len(os.listdir(recording))
    data = []
    time = []
    for i in range(length):
        timestamp = read_json(i, recording, "gametime")
        d = read_json(i, recording, key)
        time.append(float(timestamp))
        data.append(float(d))
    plt.plot(time, data)
    plt.ylabel(key)
    plt.xlabel("time [s]")
    plt.show()

if __name__=="__main__":
    data_dir = 'Saved/'
    all_recordings = os.listdir(data_dir)
    if len(all_recordings)==0:
        print("No recordings found in Saved/ folder")
        exit()

    parser = argparse.ArgumentParser()
    #default to longitudinal speed if none specified
    parser.add_argument('-k', '--key', type=str, default='car_vx')
    #default to latest recording if none specified
    parser.add_argument('-r', '--recording', type=str, default=all_recordings[-1])
    args = parser.parse_args()

    print("Plotting {} from {}".format(args.key, args.recording))
    plot_data(args.key, os.path.join(data_dir,args.recording+'/targets'))
