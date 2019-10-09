#! /usr/bin/env python

import sys
import argparse
import json


class Reindexer:
    def __init__(self):
        self.indices = {}
        self.lastIndex = -1

    def reindex(self, index):
        if index in self.indices:
            return self.indices[index]
        else:
            self.lastIndex += 1
            self.indices[index] = self.lastIndex
            return self.lastIndex


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Reordering of pose graph")
    parser.add_argument("--out_graph", dest="out_graph", type=str, required=True)
    parser.add_argument("--out_map_json", dest="out_map_json", type=str, required=True)
    args = parser.parse_args()

    indexer = Reindexer()
    out_graph = open(args.out_graph, "w")

    for line in sys.stdin.readlines():
        tokens = line.split()
        type = tokens[0]
        if type.startswith("VERTEX"):
            i1 = indexer.reindex(int(tokens[1]))
            out_graph.write("%s %d %s\n" % (type, i1, " ".join(tokens[2:])))
        elif type.startswith("EDGE"):
            i1 = indexer.reindex(int(tokens[1]))
            i2 = indexer.reindex(int(tokens[2]))
            out_graph.write("%s %d %d %s\n" % (type, i1, i2, " ".join(tokens[3:])))
        elif type == "BSPLINE_CONSTR":
            new_vertices = map(lambda t: indexer.reindex(int(t)), tokens[1:])
            format_str = "%s " + "%d "*len(new_vertices) + "\n"
            out_graph.write(format_str % tuple([type] + new_vertices))
        else:
            sys.stderr.write("Unknown element type: %s\n" % (type,))
            sys.exit(1)

    json.dump(indexer.indices, open(args.out_map_json, "w"))
