#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-

import g2o
import numpy as np
from viewer import *
import sys
class PoseGraph3D(object):
  nodes = []
  edges = []
  nodes_optimized = []
  edges_optimized = []

  def __init__(self, verbose=False):
    self.solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    self.solver=  g2o.OptimizationAlgorithmLevenberg(self.solver)

    self.optimizer = g2o.SparseOptimizer()
    self.optimizer.set_verbose(verbose)
    self.optimizer.set_algorithm(self.solver)

  def load_file(self, fname):
    self.optimizer.load(fname)
    print("vertices: ", len(self.optimizer.vertices()))
    print("edges: ", len(self.optimizer.edges()))

    for edge in self.optimizer.edges():
      self.edges.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])

    self.nodes = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])
    self.nodes_keys = self.optimizer.vertices().keys()
    for i,edge in enumerate(self.optimizer.edges()):
        print(edge.vertices())
    # print(edge.vertices())
    # self.edges_key_pairs = [ (edge.vertices().keys()[0],edge.vertices().keys()[1]) for edge in self.optimizer.edges()]
    # self.edges_key_pairs = [ print(edge) for edge in self.optimizer.edges()]


    self.nodes = np.array(self.nodes)
    self.edges = np.array(self.edges)
    print(self.nodes_keys)
    # print(self.edges_key_pairs)
  
  def optimize(self, iterations=1):
    self.optimizer.initialize_optimization()
    self.optimizer.optimize(iterations)

    self.optimizer.save("data/out.g2o")
    self.edges_optimized = []
    for edge in self.optimizer.edges():
      self.edges_optimized.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])

    self.nodes_optimized = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])
    self.nodes_optimized = np.array(self.nodes_optimized)
    self.edges_optimized = np.array(self.edges_optimized)

if __name__ == "__main__":
  if len(sys.argv) > 1:
    gfile = str(sys.argv[1])
  else:
    # gfile = "/home/jiangpin/dataset/example_4robots/two_stage_centralized.g2o"
    gfile = "./data/sphere2500.g2o"

    


  graph = PoseGraph3D()
  graph.load_file(gfile)
  #graph.optimize()
  print("loaded")
  # viewer = Viewer3D(graph)

  # graph.optimize()
  # viewer = Viewer3D(graph)




