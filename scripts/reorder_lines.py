#! /usr/bin/env python

import sys
import argparse
import json


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Reordering of lines")
    parser.add_argument("--map_json", dest="map_json", type=str, required=True)
    args = parser.parse_args()

    lines = sys.stdin.readlines()

    map_indices_str = json.load(open(args.map_json))
    map_indices = {int(k): int(v) for k, v in map_indices_str.items()}
    for i in sorted(map_indices.keys()):
        print lines[map_indices[i]].strip()
