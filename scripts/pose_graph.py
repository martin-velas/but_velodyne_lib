#! /usr/bin/env python

class PrecisionMatrix:
    def __init__(self, diagonal_):
        assert len(diagonal_) == 3 or len(diagonal_) == 6
        self.diagonal = diagonal_

    def to_upper_matrix(self):
        if len(self.diagonal) == 3:
            return [self.diagonal[0], 0.0, 0.0,
                         self.diagonal[1], 0.0,
                              self.diagonal[2]]
        elif len(self.diagonal) == 6:
            return [self.diagonal[0], 0.0, 0.0, 0.0, 0.0, 0.0,
                         self.diagonal[1], 0.0, 0.0, 0.0, 0.0,
                              self.diagonal[2], 0.0, 0.0, 0.0,
                                   self.diagonal[3], 0.0, 0.0,
                                        self.diagonal[4], 0.0,
                                             self.diagonal[5]]
        else:
            assert False

    def __str__(self):
        output = ""
        for m in self.to_upper_matrix():
            output += "%s " % m
        return output

class Edge3D:
    def __init__(self, src_id, trg_id, dof6, inf_matrix_diagonal=[100]*6):
        self.srcId = src_id
        self.trgId = trg_id
        self.dof6 = dof6
        self.precision_matrix = PrecisionMatrix(inf_matrix_diagonal)

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
        output += str(self.precision_matrix)
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


class BsplineEdge:
    def __init__(self, n0_, n1_, n2_, n3_, t_, n_ref_, inf_matrix_diagonal_):
        self.n0 = n0_
        self.n1 = n1_
        self.n2 = n2_
        self.n3 = n3_
        self.t = t_
        self.n_ref = n_ref_
        self.inf_matrix_diagonal = tuple(inf_matrix_diagonal_)

    def __str__(self):
        return ("BSPLINE_CONSTR %d %d %d %d %f %d" + " %s"*6) % \
               ((self.n0, self.n1, self.n2, self.n3, self.t, self.n_ref) + self.inf_matrix_diagonal)

class EdgeToFeature:
    def __init__(self, src_i_, feat_i_, translation_, inf_matrix_diagonal_):
        self.src_i = src_i_
        self.feat_i = feat_i_
        self.t = translation_
        self.precision_matrix = PrecisionMatrix(inf_matrix_diagonal_)

    def __str__(self):
        return "EDGE_SE3_XYZ %d %d %f %f %f %s" % \
               (self.src_i, self.feat_i, self.t[0], self.t[1], self.t[2], self.precision_matrix)
