#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-
import sys
import pickle

import g2o
import numpy as np
import re
# from sophus import *

# from viewer import Viewer3D
# from multi_viewer import MultiViewer3D
from multi_robot_tools import MultiRobotTools
from g2o_tool import G2oTool


class PoseGraph3D(object):
  nodes = []
  edges = []
  nodes_optimized = []
  edges_optimized = []

  def __init__(self, verbose=False,robot_id = 0,use_transform=False):
    self.solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())
    self.solver=  g2o.OptimizationAlgorithmLevenberg(self.solver)
    self.robot_id = robot_id
    self.use_transform = use_transform

    self.optimizer = g2o.SparseOptimizer()
    self.optimizer.set_verbose(verbose)
    self.optimizer.set_algorithm(self.solver)
    self.g2o_tool = G2oTool()
    self.multi_robot_tools = MultiRobotTools()


  def load_file(self, fname):
    self.optimizer.load(fname)
    # print(dir(self.optimizer))

    print("vertices: ", len(self.optimizer.vertices()))
    print("edges: ", len(self.optimizer.edges()))

    # self.edges_key_pairs = self.g2o_tool.read_edge_keys(fname)
    self.edges_key_pairs = []
    self.edges = []
    self.key_node_dict = {}

    if (self.use_transform):
      self.init_transform(fname)
      for edge in self.optimizer.edges():
        robot_id0 = self.g2o_tool.key2robot_id_g2o(edge.vertices()[0].id())
        robot_id1 = self.g2o_tool.key2robot_id_g2o(edge.vertices()[1].id())

        node0 = np.matmul(self.transform_list[robot_id0],edge.vertices()[0].estimate().matrix())
        node1 = np.matmul(self.transform_list[robot_id1],edge.vertices()[1].estimate().matrix())

        self.edges.append([node0,node1])
        self.edges_key_pairs.append([edge.vertices()[0].id(),edge.vertices()[1].id()])

      self.nodes = []
      self.key_node_dict = {}

      for key,value in zip(self.optimizer.vertices().keys(),self.optimizer.vertices().values()):
          robot_id = self.g2o_tool.key2robot_id_g2o(key)
          node = np.matmul(self.transform_list[robot_id],value.estimate().matrix())
          position = [node[0,3],node[1,3],node[2,3]]
          self.key_node_dict[key] = position
          self.nodes.append(node)
      
    else:
      # print(self.optimizer.edges())
      for edge in self.optimizer.edges():
        try:
          # print(edge.vertices()[0].id(),edge.vertices()[1].id())
          self.edges.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])
          self.edges_key_pairs.append([edge.vertices()[0].id(),edge.vertices()[1].id()])
        except:
          print("none")
      self.update_key_position()
      self.nodes = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])


    self.nodes_keys = [key for key in self.optimizer.vertices().keys()]

   
    
    self.nodes = np.array(self.nodes)
    self.edges = np.array(self.edges)
    self.nodes_keys = np.array(self.nodes_keys)
    self.edges_key_pairs = np.array(self.edges_key_pairs)
    self.init_separator()
    # print(self.separator_nodes.shape)

    # print(len(self.edges_key_pairs))
    
    # print(self.nodes_keys)
    # print(self.edges_key_pairs)

    # self.nodes_keys = np.array(self.nodes_keys)
    # print(self.nodes_keys)
    # print(self.edges_key_pairs)

  def init_transform(self,fname):
    self.data_dir = self.extract_dir(fname)

    transform_name = self.data_dir+'/full_graph_transform.pkl'
    print(transform_name)
    with open(transform_name,'rb') as f:
      transform_list = pickle.load(f)

    self.transform_list = [np.array(each) for each in transform_list]

  def extract_dir(self,fname):
    fname = fname.split('/')
    dir_name = ''
    for ch in fname[1:-1]:
      dir_name = dir_name+'/'+ch
    return dir_name

  def update_key_position(self):
     for key,value in zip(self.optimizer.vertices().keys(),self.optimizer.vertices().values()):
      node = value.estimate().matrix()
      position = [node[0,3],node[1,3],node[2,3]]
      self.key_node_dict[key] = position
  
  def initial_guess(self):
    self.optimizer.initialize_optimization()
    self.edges_optimized = []
    for edge in self.optimizer.edges():
      self.edges_optimized.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])

    self.nodes_optimized = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])
    self.nodes_optimized = np.array(self.nodes_optimized)
    self.edges_optimized = np.array(self.edges_optimized)
    
  def optimize(self, iterations=1):
    self.optimizer.optimize(iterations)

    self.optimizer.save("data/out.g2o")
    self.edges_optimized = []
    for edge in self.optimizer.edges():
      self.edges_optimized.append([edge.vertices()[0].estimate().matrix(), edge.vertices()[1].estimate().matrix()])

    self.nodes_optimized = np.array([i.estimate().matrix() for i in self.optimizer.vertices().values()])
    self.nodes_optimized = np.array(self.nodes_optimized)
    self.edges_optimized = np.array(self.edges_optimized)

    # self.nodes = self.nodes_optimized
    # self.edges = self.edges_optimized

  def init_separator(self):
    separator_key = np.array( [key_pair for key_pair in self.edges_key_pairs if (self.multi_robot_tools.key2robot_id_g2o(key_pair[0])!=self.multi_robot_tools.key2robot_id_g2o(key_pair[1])) ] )
    separator_edge_mask = []

    separator_edge_mask = np.array([ (self.multi_robot_tools.is_separator_g2o(key_pair)) for key_pair in self.edges_key_pairs])

    separator_node_key = []
    for key_pair in separator_key:
      separator_node_key.append(key_pair[0])
      separator_node_key.append(key_pair[1])
    separator_node_key = np.array(separator_node_key)
    separator_node_key = np.unique(separator_node_key)

    separator_node_mask = np.isin(self.nodes_keys,separator_node_key)

    # print(separator_node_mask)
    self.separator_edges = np.array(self.edges[separator_edge_mask])
    self.separator_nodes = np.array(self.nodes[separator_node_mask])

  def linearize(self,poses,linearize_point):
    R_ref = np.matrix(linearize_point[0:3,0:3])
    R_ref_inv = R_ref.getI()
    linearize_pose = []
    for pose in poses:
      rot = np.matrix(pose[0:3,0:3])
      rot_ref = np.dot(rot,R_ref_inv)
      linearize_rot = np.array(SO3(rot_ref).log())
      linearize_pose.append(np.concatenate(linearize_point[0:2,3],linearize_rot))
    linearize_pose = np.array(linearize_pose)
    
def is_file_renamed(filename):
  sub_string = 'renamed'
  pattern = re.compile(sub_string)
  if pattern.search(filename):
    return True
  else:
    return False

if __name__ == "__main__":
  print("main")




