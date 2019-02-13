#! /usr/bin/env python

import sys, argparse

parser = argparse.ArgumentParser(description="join on common field")
parser.add_argument("-s", "--src_file", dest="src_file", type=str, required=True)
parser.add_argument("--src_index", dest="src_index", type=int, required=False, default=0)
parser.add_argument("-t", "--trg_file", dest="trg_file", type=str, required=True)
parser.add_argument("--trg_index", dest="trg_index", type=int, required=False, default=0)
parser.add_argument("-d", "--delimiter", dest="delim", type=int, required=False, default=None)
args = parser.parse_args()

src_lines = open(args.src_file).readlines()
src_dict = {l.split(args.delim)[args.src_index]: l.strip() for l in src_lines}

output_delim = args.delim if args.delim is not None else " "
for trg_line in open(args.trg_file).readlines():
    trg_line = trg_line.strip()
    trg_key = trg_line.split(args.delim)[args.trg_index]
    if trg_key in src_dict:
        print output_delim.join([src_dict[trg_key], trg_line])
