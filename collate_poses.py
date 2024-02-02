"""
collate aligned SE3 poses into one text file
"""

import argparse
import os

def parse_args():
    parser = argparse.ArgumentParser(description="collate aligned SE3 poses into one text file")
    parser.add_argument("trans_dir", type=str, help="dir containing the transformation text files")
    parser.add_argument("output_file", type=str, help="output file to save the collated poses")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    transfiles = []
    for file in os.listdir(args.trans_dir):
        if file.endswith(".txt"):
            transfiles.append(os.path.join(args.trans_dir, file))
    transfiles = sorted(transfiles, key=lambda i: int(os.path.splitext(os.path.basename(i))[0]))
    count = 0
    with open(args.output_file, "a") as f:
        for file in transfiles:
            print("collating poses from {}".format(file))
            with open(file, "r") as g:
                lines = g.readlines()
                for line in lines:
                    if line.strip() != "":
                        f.write(line)
                        count += 1

    assert count == len(transfiles), "number of poses collated does not match number of files"
    print("collated {} poses".format(count))
