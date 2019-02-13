#! /usr/bin/env python

import sys
import copy
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


class GraphEnvelope:
    maxDistanceForBackward = 3

    def __init__(self, start_idx, end_id, all_edges):
        self.start = start_idx
        self.end = end_id
        self.edges = defaultdict(list)
        self.vertices = []
        for e in all_edges.keys():
            if e.srcIdx < self.end and e.trgIdx > self.start and (e.srcIdx != self.start or e.trgIdx != self.end):
                self.edges[e.srcIdx].append(e.trgIdx)
                self.edges[e.trgIdx].append(e.srcIdx)
                self.vertices.append(e.srcIdx)
                self.vertices.append(e.trgIdx)

    def empty(self):
        return len(self.vertices) == 0

    def findPaths(self):
        self.allPaths = []
        visited = [False]*(max(self.vertices)+1)
        self.findPathsRecursive(self.start, visited, [], self.end - self.start <= self.maxDistanceForBackward)
        return self.allPaths

    def findPathsRecursive(self, current, visited, path, backward):
        visited[current] = True
        path.append(current)

        if current == self.end:
            self.allPaths.append(copy.deepcopy(path))
        else:
            for next in self.edges[current]:
                if not visited[next] and (current < next or backward):
                    self.findPathsRecursive(next, visited, path, (current < next and backward))

        path.pop()
        visited[current] = False


def compute_connectivity(edges, verticles_cnt):
    connectivity = [0]*verticles_cnt
    for e in edges.keys():
        for i in range(e.srcIdx, e.trgIdx+1):
            connectivity[i] += 1
    return connectivity


def path_transformation(edges, path):
    T = np.eye(4)
    for src_i, trg_i in zip(path[:len(paths)-1], path[1:]):
        swap = src_i > trg_i
        if swap:
            src_i, trg_i = trg_i, src_i
        Ti = edges[EdgeKey(src_i, trg_i)].M
        if swap:
            Ti = np.linalg.inv(Ti)
        T = np.dot(T, Ti)
    return T


def load_edges(file):
    edges = {}
    for line in file.readlines():
        tokens = line.split()
        src_i, trg_i = int(tokens[0]), int(tokens[1])
        T = map(float, tokens[2:14])
        edges[EdgeKey(src_i, trg_i)] = Odometry(T)
    return edges


if len(sys.argv) == 1:
    sys.stderr.write("ERROR, usage: %s [vertices.connectivity] [edges.error]\n" % (sys.argv[0],))
    sys.exit(1)

edges = load_edges(sys.stdin)
max_vertex = max(sum([[e.srcIdx, e.trgIdx] for e in edges], []))

if len(sys.argv) > 1:
    connectivity = compute_connectivity(edges, max_vertex + 1)
    with open(sys.argv[1], "w") as connectivity_file:
        for c in connectivity:
            connectivity_file.write("%d\n" % (c,))

if len(sys.argv) > 2:
    with open(sys.argv[2], "w") as precision_file:
        for e in edges:
            subgraph = GraphEnvelope(e.srcIdx, e.trgIdx, edges)
            paths = subgraph.findPaths() if not subgraph.empty() else []
            sum_errors = 0.0
            Te = edges[e].M
            for p in paths:
                Tp = path_transformation(edges, p)
                sum_errors += np.linalg.norm(Te[0:3, 3] - Tp[0:3, 3])
            if len(paths) == 0:
                error = -1.0
            else:
                error = sum_errors/len(paths)
            precision_file.write("%d %d %f\n" % (e.srcIdx, e.trgIdx, error))
