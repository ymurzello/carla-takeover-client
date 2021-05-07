import json
import argparse
from os import listdir
from os.path import isfile, join
import pandas as pd

def main(args):
    
    # Perform some checks
    if len(args.json_dir) == 0:
        print('Missing argument: --json_dir')

    if len(args.csv_out) == 0:
        print('Missing argument: --csv_out')

    content_dict = dict()

    for file_path in listdir(args.json_dir):
        file_path = join(args.json_dir, file_path)
        if isfile(file_path) :
            append_json_to_dict(file_path, content_dict)

    df = pd.DataFrame.from_dict(content_dict)
    df.to_csv(args.csv_out)


def append_json_to_dict(json_path, content_dict):
    json_data = json.load(open(json_path))
    
    if len(content_dict.keys()) == 0:
        #If the dict is empty, init all the keys
        
        #Init some key to keep track of original json files
        content_dict["original_json"] = list()

        for key in json_data.keys(): 
            content_dict[key] = list()

    content_dict["original_json"].append(json_path)

    for key in json_data.keys(): 
        if key in content_dict:
            content_dict[key].append(json_data[key])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--json_dir', type=str, default="")
    parser.add_argument('--csv_out', type=str, default="")
    args = parser.parse_args()

    try:
        main(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

