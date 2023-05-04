#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-
from multiprocessing import Process, Queue

import pangolin as pango
import numpy as np
import OpenGL.GL as gl

from multi_robot_tools import MultiRobotTools

class MultiViewer3D(object):
  '''
  3d viewer for g2o maps
    - based off ficiciSLAM's viewer
       - github.com/kemfic/ficiciSLAM
  '''
  is_optim = False
  tform = np.array([[0.0, 0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])
  def __init__(self, graph,robot_num,file_name='g2o_stuff'):
    self.file_name = file_name
    self.init()
    self.color_init()
    self.graph = graph
    self.nodes = np.dot(graph.nodes, self.tform)
    self.edges = np.array(graph.edges)
    self.robot_num = robot_num
    self.nodes_keys = np.array(graph.nodes_keys)
    self.edges_key_pairs = np.array(graph.edges_key_pairs)
    self.multi_robot_tools = MultiRobotTools()
    self.separator_edges = self.graph.separator_edges 
    self.separator_nodes = np.dot(self.graph.separator_nodes,self.tform)
    self.partition_graph_np()
    while not pango.ShouldQuit():
      self.refresh()

  def init(self):
    w, h = (1024,768)
    f = 2000 #420

    pango.CreateWindowAndBind(self.file_name , w, h)
    gl.glEnable(gl.GL_DEPTH_TEST)
    self.camera_size=0.2
    # Projection and ModelView Matrices
    self.scam = pango.OpenGlRenderState(
        pango.ProjectionMatrix(w, h, f, f, w //2, h//2, 0.1, 100000),
        pango.ModelViewLookAt(0, -50.0, -10.0,
                              0.0, 0.0, 0.0,
                              0.0, -1.0, 0.0))#pango.AxisDirection.AxisY))
    self.handler = pango.Handler3D(self.scam)

    # Interactive View in Window
    self.dcam = pango.CreateDisplay()
    self.dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -w/h)
    panel = pango.CreatePanel('ui')
    panel.SetBounds(0.0, 1.0, 0.0, 180/640.)
    # self.rename_id = pango.VarString('ui.rename id', "10")
    # self.rename_id_dir = pango.VarString('ui.rename id dir', "10")

    self.reset_view_button = pango.VarBool('ui.Reset View', value=False, toggle=False)

    self.show_separator = pango.VarBool('ui.Show Separator', value=True, toggle=True)
    self.show_separator_only = pango.VarBool('ui.Only Show Separator', value=False, toggle=True)
    self.show_robot_1 = pango.VarBool('ui.Show Robot 1', value=True, toggle=True)
    self.show_robot_2 = pango.VarBool('ui.Show Robot 2', value=True, toggle=True)
    self.show_robot_3 = pango.VarBool('ui.Show Robot 3', value=True, toggle=True)
    self.show_robot_4 = pango.VarBool('ui.Show Robot 4', value=True, toggle=True)

    self.rename_id_button = pango.VarBool('ui.Renameid', value=False, toggle=False)
    self.rename_id_dir_button = pango.VarBool('ui.Renameid Dir', value=False, toggle=False)

    self.iterations = pango.VarString('ui.Iterations', "10")
    self.init_guess_button = pango.VarBool('ui.Initial Guess', value=False, toggle=False)
    self.reload_button = pango.VarBool('ui.Reload', value=False, toggle=False)
    self.optimize_button = pango.VarBool('ui.Optimize', value=False, toggle=False)

  



    self.dcam.SetHandler(self.handler)
    self.dcam.Activate()

    pango.RegisterKeyPressCallback(ord('r'), self.optimize_callback)
    pango.RegisterKeyPressCallback(ord('t'), self.switch_callback)

  def color_init(self):
    color_list = [[255,69,0],[255,215,0],[0,255,127],[0,191,255],[138,43,226]]
    self.color_list = color_list
    self.separator_edge_color = [244,164,96]
    self.separator_node_color = [255,250,205]

  def refresh(self):

    if pango.Pushed(self.reset_view_button):
      print("reset view")
    if pango.Pushed(self.rename_id_button):
      print("rename id")
    if pango.Pushed(self.rename_id_dir_button):
      print("rename id dir")
    if pango.Pushed(self.init_guess_button):
      print("init_guess_button")
      self.init_guess_callback()
    if pango.Pushed(self.reload_button):
      print("reload")
    if pango.Pushed(self.optimize_button):
      print("optimize")
      self.optimize_button_callback()

    

    #clear and activate screen
    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
    gl.glClearColor(0.15, 0.15, 0.15, 0.0)
    #gl.glClearColor(1.0, 1.0, 1.0, 0.0)

    self.dcam.Activate(self.scam)
    #draw separator
    # if len(self.separator_nodes) >1:
    #   gl.glLineWidth(2)
    #   gl.glColor3f(self.separator_node_color[0]/255.0, self.separator_node_color[1]/255.0, self.separator_node_color[2]/255.0)
    #   pango.DrawCameras(self.separator_nodes,self.camera_size)
      
    # if len(self.separator_edges) >1:
    #   gl.glLineWidth(3)
    #   gl.glColor3f(self.separator_edge_color[0]/255.0, self.separator_edge_color[1]/255.0, self.separator_edge_color[2]/255.0)
    #   pango.DrawLines(self.separator_edges[:,0,:-1, -1], self.separator_edges[:,1,:-1,-1])

    for robot_id in range(self.robot_num):
      edge_color = self.color_list[robot_id]
      nodes = self.nodes_dict[robot_id]
      edges = self.edges_dict[robot_id]

      # render
      # render cameras
      gl.glLineWidth(1)
      if len(nodes) > 1:
        gl.glColor3f(edge_color[0]/255.0, edge_color[1]/255.0, edge_color[2]/255.0)
        # gl.glColor3f(1.0, 1.0, 1.0)

        pango.DrawCameras(nodes,self.camera_size)
      
      # render edges
      if len(edges) > 1:
        gl.glColor3f(edge_color[0]/255.0, edge_color[1]/255.0, edge_color[2]/255.0)
        pango.DrawLines(edges[:,0,:-1, -1], edges[:,1,:-1,-1])

   
    pango.FinishFrame()

  def partition_graph_np(self):
    self.nodes_dict = {}
    self.edges_dict = {}

    for robot_id in range(self.robot_num):
      node_id_mask = np.array([self.multi_robot_tools.key2robot_id_g2o(key)==robot_id for key in self.nodes_keys])
      # node_id_mask = np.array([self.multi_robot_tools.key2robot_id_g2o(key) for key in self.nodes_keys])
      # node_id_mask = np.array([key for key in self.nodes_keys])

      # print(node_id_mask)
      
      self.nodes_dict[robot_id] = self.nodes[node_id_mask]

      node_key = self.nodes_keys[node_id_mask]

      # print("----------node_key---------------")
      # print("----------robot"+str(robot_id)+'-----------')
      # print(node_key)


      edge_id_mask = np.array([ (self.multi_robot_tools.key2robot_id_g2o(key_pair[0]) ==robot_id or self.multi_robot_tools.key2robot_id_g2o(key_pair[1])==robot_id)
                               for key_pair in self.edges_key_pairs])

      edges_key_pairs = self.edges_key_pairs[edge_id_mask]

      # print("----------edge_key---------------")
      # print("----------robot"+str(robot_id)+'-----------')
      # print(edges_key_pairs)

      self.edges_dict[robot_id] = self.edges[edge_id_mask]


  def update(self, graph=None):
    '''
    add new stuff to queues
    '''
    
    self.nodes = np.dot(self.graph.nodes_optimized, self.tform)
    self.edges = self.graph.edges_optimized


  def optimize_callback(self):
    self.graph.optimize()
    self.is_optim = False
    self.switch_callback()
  def switch_callback(self):
    self.is_optim = ~self.is_optim
    if self.is_optim:
      print("optimized")
      self.nodes = np.dot(self.graph.nodes_optimized, self.tform)
      self.edges = self.graph.edges_optimized
    else:
      print("original")
      self.nodes = np.dot(self.graph.nodes, self.tform)
      self.edges = self.graph.edges

  def init_guess_callback(self):
    self.graph.initial_guess()
    self.update()
    self.partition_graph_np()

  def optimize_button_callback(self):
    self.graph.optimize(int(self.iterations.Get()))
    self.update()
    self.partition_graph_np()
