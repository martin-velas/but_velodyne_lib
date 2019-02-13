#! /usr/bin/env python

import sys


class Vertex:
    def __init__(self, type, idx, rest):
        self.type = type
        self.idx = idx
        self.rest = rest

    def __str__(self):
        return "%s %d %s" % (self.type, self.idx, self.rest)

    def __lt__(self, other):
        try:
            return self.idx < other.idx
        except AttributeError:
            return self.idx < other.trgIdx


class Edge:
    def __init__(self, type, src_i, trg_i, rest):
        self.type = type
        assert src_i <= trg_i
        self.srcIdx = src_i
        self.trgIdx = trg_i
        self.rest = rest

    def __str__(self):
        return "%s %d %d %s" % (self.type, self.srcIdx, self.trgIdx, self.rest)

    def __lt__(self, other):
        try:
            if self.trgIdx == other.trgIdx:
                return self.srcIdx < other.srcIdx
            else:
                return self.trgIdx < other.trgIdx
        except AttributeError:
            return self.trgIdx < other.idx


elements = []
for line in sys.stdin.readlines():
    tokens = line.split()
    type = tokens[0]
    if type.startswith("VERTEX"):
        elements.append(Vertex(type, int(tokens[1]), " ".join(tokens[2:])))
    elif type.startswith("EDGE"):
        elements.append(Edge(type, int(tokens[1]), int(tokens[2]), " ".join(tokens[3:])))
    else:
        sys.stderr.write("Unknown element type: %s\n" % (type,))

elements.sort()
for e in elements:
    print e
