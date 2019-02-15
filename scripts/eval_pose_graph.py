#! /usr/bin/env python

import sys
import copy
import argparse
from odometry_cnn_data import Odometry
from collections import defaultdict
import numpy as np


class EdgeKey:

    def __init__(self, src_i, trg_i):
        self.srcIdx = src_i
        self.trgIdx = trg_i

    def __str__(self):
        return str([self.srcIdx, self.trgIdx])

    def __repr__(self):
        return str(self)

    def __lt__(self, other):
        if self.srcIdx == other.srcIdx:
            return self.trgIdx < other.trgIdx
        else:
            return self.srcIdx < other.srcIdx

    def __eq__(self, other):
        return (self.srcIdx == other.srcIdx) and (self.trgIdx == other.trgIdx)

    def __hash__(self):
        return self.srcIdx*1000000 + self.trgIdx


def vertex_in_range(vertex, start, end, vertices_cnt, circular):
    if not circular or (end - start) < vertices_cnt/2:
        return start <= vertex <= end
    else:
        return 0 <= vertex <= start or end <= vertex < vertices_cnt


def compute_connectivity(edges, verticles_cnt, circular):
    connectivity = [0]*verticles_cnt
    for e in edges.keys():
        for i in range(verticles_cnt):
            if vertex_in_range(i, e.srcIdx, e.trgIdx, verticles_cnt, circular):
                connectivity[i] += 1
    return connectivity


class GraphEnvelope:
    maxDepth = 4
    allPaths = []

    def __init__(self, start_idx, end_idx, all_edges, vertices_cnt, circular):
        self.start = start_idx
        self.end = end_idx
        self.edges = defaultdict(list)
        self.verticesCnt = vertices_cnt
        for e in all_edges.keys():
            if self.hasOverlapWith(e, circular) and (e.srcIdx != self.start or e.trgIdx != self.end):
                self.edges[e.srcIdx].append(e.trgIdx)
                self.edges[e.trgIdx].append(e.srcIdx)

    def hasOverlapWith(self, edge, circular):
        return vertex_in_range(edge.srcIdx, self.start, self.end, self.verticesCnt, circular) or \
               vertex_in_range(edge.trgIdx, self.start, self.end, self.verticesCnt, circular)

    def empty(self):
        return len(self.edges) == 0

    def findPaths(self):
        self.allPaths = []
        visited = [False]*self.verticesCnt
        self.findPathsRecursive(self.start, visited, [])
        return self.allPaths

    def findPathsRecursive(self, current, visited, path):
        visited[current] = True
        path.append(current)

        if current == self.end:
            self.allPaths.append(copy.deepcopy(path))
        elif len(path) < self.maxDepth:
            for next in self.edges[current]:
                if not visited[next]:
                    self.findPathsRecursive(next, visited, path)

        path.pop()
        visited[current] = False


def path_transformation(edges, path):
    T = np.eye(4)
    for src_i, trg_i in zip(path[:len(paths)-1], path[1:]):
        swap = src_i > trg_i
        if swap:
            src_i, trg_i = trg_i, src_i
        if not swap:
            Ti = edges[EdgeKey(src_i, trg_i)][0]
        else:
            Ti = edges[EdgeKey(src_i, trg_i)][1]
        T = np.dot(T, Ti)
    return T


def load_edges(file):
    edges = {}
    for line in file.readlines():
        tokens = line.split()
        src_i, trg_i = int(tokens[0]), int(tokens[1])
        T = Odometry(map(float, tokens[2:14])).M
        edges[EdgeKey(src_i, trg_i)] = (T, np.linalg.inv(T))
    return edges


parser = argparse.ArgumentParser(description="Evaluation of the registrations (future edges in pose graph)")
parser.add_argument("-c", "--out_connectivity", dest="connectivity_fn", type=str, required=False)
parser.add_argument("-e", "--out_error", dest="error_fn", type=str, required=False)
parser.add_argument("--circular", dest="circular", default=False, action="store_true")
args = parser.parse_args()

edges = load_edges(sys.stdin)
vertices_cnt = max(sum([[e.srcIdx, e.trgIdx] for e in edges], [])) + 1

if args.connectivity_fn is not None:
    connectivity = compute_connectivity(edges, vertices_cnt, args.circular)
    with open(args.connectivity_fn, "w") as connectivity_file:
        for c in connectivity:
            connectivity_file.write("%d\n" % (c,))

if args.error_fn is not None:
    with open(args.error_fn, "w") as error_file:
        for e in edges:
            subgraph = GraphEnvelope(e.srcIdx, e.trgIdx, edges, vertices_cnt, args.circular)
            paths = subgraph.findPaths() if not subgraph.empty() else []
            errors = []
            Te = edges[e][0]
            for p in paths:
                Tp = path_transformation(edges, p)
                errors.append(np.linalg.norm(Te[0:3, 3] - Tp[0:3, 3]))
            if len(errors) == 0:
                error = -1.0
            else:
                error = np.median(errors)
            error_file.write("%d %d %f\n" % (e.srcIdx, e.trgIdx, error))
