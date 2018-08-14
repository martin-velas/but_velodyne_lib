#! /usr/bin/env python

class Edge3D:
    def __init__(self, src_id, trg_id, dof6, inf_matrix_diagonal = None):
        self.srcId = src_id
        self.trgId = trg_id
        self.dof6 = dof6
        if inf_matrix_diagonal is None:
            self.inf_matrix_upper_half = [99.1304, -0.869565, -0.869565, -1.73913, -1.73913, -1.73913,
                                                    99.13040, -0.869565, -1.73913, -1.73913, -1.73913,
                                                               99.13050, -1.73913, -1.73913, -1.73913,
                                                                          96.5217, -3.47826, -3.47826,
                                                                                    96.5217, -3.47826,
                                                                                             96.52170]
        else:
            self.inf_matrix_upper_half = [inf_matrix_diagonal[0], 0.0, 0.0, 0.0, 0.0, 0.0,
                                               inf_matrix_diagonal[1], 0.0, 0.0, 0.0, 0.0,
                                                    inf_matrix_diagonal[2], 0.0, 0.0, 0.0,
                                                         inf_matrix_diagonal[3], 0.0, 0.0,
                                                              inf_matrix_diagonal[4], 0.0,
                                                                   inf_matrix_diagonal[5]]

    def __gt__(self, other):
        if self.srcId > other.srcId:
            return True
        elif self.srcId < other.srcId:
            return False
        else:
            return self.trgId > other.trgId

    def __str__(self):
        output = "EDGE3 %s %s " % (self.srcId, self.trgId)
        for d in self.dof6:
            output += "%s " % d
        for m in self.inf_matrix_upper_half:
            output += "%s " % m
        return output


def get_max_vertex(graph_file):
    max_vertex = -1
    for line in open(graph_file).readlines():
        tokens = line.split()
        src_vertex = int(tokens[1])
        trg_vertex = int(tokens[2])
        max_vertex = max(max_vertex, max(src_vertex, trg_vertex))
    return max_vertex


class EdgesGenerator:
    def __init__(self, new_poses, original_poses, reg_pose, graph_file):
        self.new_poses = new_poses
        self.original_poses = original_poses
        self.reg_pose = reg_pose
        self.max_vertex = get_max_vertex(graph_file)

    def genEdgesToNewVertex(self, idx_from, idx_to):
        self.max_vertex = new_vertex = self.max_vertex+1
        edges = []
        for i in range(idx_from, idx_to+1):
            t = self.original_poses[i].inv() * self.new_poses[0]
            edges.append(Edge3D(i, new_vertex, t.dof))
        return edges

    def genEdges(self, src_idx_from, src_idx_to, trg_idx_from, trg_idx_to):
        edges  = self.genEdgesToNewVertex(src_idx_from, src_idx_to)
        edges += self.genEdgesToNewVertex(trg_idx_from, trg_idx_to)
        edges.append(Edge3D(self.max_vertex-1, self.max_vertex, self.reg_pose.dof))
        return edges


def print_first_vertex(dof, inf_matrix_diagonal):
    print "VERTEX3 -1 0 0 0 0 0 0"
    print Edge3D(-1, 0, dof, inf_matrix_diagonal)
