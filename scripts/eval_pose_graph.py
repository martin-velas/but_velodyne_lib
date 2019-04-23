#! /usr/bin/env python

import sys
import copy
import argparse
from odometry_cnn_data import Odometry
from collections import defaultdict
import numpy as np
import json


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

    def increasing(self):
        return self.srcIdx <= self.trgIdx

    def swap(self):
        return EdgeKey(self.trgIdx, self.srcIdx)


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


class Graph:
    maxDepth = 4
    allPaths = []

    def __init__(self, all_edges):
        self.edges = defaultdict(list)
        self.verticesCnt = -1
        for e in all_edges.keys():
            self.verticesCnt = max([e.srcIdx, e.trgIdx, self.verticesCnt])
            self.edges[e.srcIdx].append(e.trgIdx)
            self.edges[e.trgIdx].append(e.srcIdx)
        self.verticesCnt += 1

    def empty(self):
        return len(self.edges) == 0

    def findPaths(self, edge):
        self.allPaths = []
        visited = [False]*self.verticesCnt
        self.findPathsRecursive(edge.srcIdx, edge.trgIdx, visited, [])
        return self.allPaths

    def findPathsRecursive(self, current, target, visited, path):
        visited[current] = True
        path.append(current)

        if current == target:
            self.allPaths.append(copy.deepcopy(path))
        elif len(path) < self.maxDepth:
            for next in self.edges[current]:
                if not visited[next]:
                    self.findPathsRecursive(next, target, visited, path)

        path.pop()
        visited[current] = False


def path_transformation(edges, path):
    T = np.eye(4)
    for e in path2edges(path):
        if e.increasing():
            Ti = edges[e][0]
        else:
            Ti = edges[e.swap()][1]
        T = np.dot(T, Ti)
    return T


def path2edges(path):
    src_trg_pairs = zip(path[:len(paths) - 1], path[1:])
    return [EdgeKey(src_i, trg_i) for src_i, trg_i in src_trg_pairs]


def load_edges(file):
    edges = {}
    for line in file.readlines():
        tokens = line.split()
        src_i, trg_i = int(tokens[0]), int(tokens[1])
        T = Odometry(map(float, tokens[2:14])).M
        edges[EdgeKey(src_i, trg_i)] = (T, np.linalg.inv(T))
    return edges


if __name__ == "__main__":

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

    edge_errors = defaultdict(list)
    graph = Graph(edges)
    if args.error_fn is not None:
        for e in edges:
            paths = graph.findPaths(e)
            Te = edges[e][0]
            for p in paths:
                Tp = path_transformation(edges, p)
                err = np.linalg.norm((np.linalg.inv(Te)*Tp)[0:3, 3])
                edge_errors[e].append(err)
                for adj_edge in path2edges(p):
                    if not adj_edge.increasing():
                        adj_edge = adj_edge.swap()
                    edge_errors[adj_edge].append(err)

        with open(args.error_fn + ".json", "w") as errors_json_file:
            json_edge_errors = {}
            for e in edge_errors:
                json_edge_errors[str(e.srcIdx) + " " + str(e.trgIdx)] = edge_errors[e]
            json.dump(json_edge_errors, errors_json_file)

        with open(args.error_fn, "w") as error_file:
            for e in edges:
                if e in edge_errors:
                    error = np.median(edge_errors[e])
                else:
                    error = -1.0
                error_file.write("%d %d %f\n" % (e.srcIdx, e.trgIdx, error))
