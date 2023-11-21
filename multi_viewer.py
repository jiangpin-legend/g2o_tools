#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-
from multiprocessing import Process, Queue

import pangolin as pango
import numpy as np
import OpenGL.GL as gl
import OpenGL.GLUT as glut

import tkinter as tk
from tkinter import filedialog
from PyQt5.QtWidgets import QFileDialog,QMainWindow

from g2o_tool import G2oTool
from posegraph import is_file_renamed
from posegraph import PoseGraph3D
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

    root = tk.Tk()
    root.withdraw()

    while not pango.ShouldQuit():
      self.refresh()

  
  def init(self):
    w, h = (1024,768)
    f = 2000 #420

    pango.CreateWindowAndBind(self.file_name , w, h)
    gl.glEnable(gl.GL_DEPTH_TEST)
    glut.glutInit()
    self.camera_size=0.2
    self.viewpoint_x = 0
    self.viewpoint_y = 0  
    self.viewpoint_z = 100
    self.viewpoint_f = 1000
    # Projection and ModelView Matrices
    self.look_view = pango.ModelViewLookAt(self.viewpoint_x, self.viewpoint_y, self.viewpoint_z,
                              0.0, 0.0, 0.0,
                              0.0, -1.0, 0.0)
    self.scam = pango.OpenGlRenderState(
        pango.ProjectionMatrix(w, h, self.viewpoint_f, self.viewpoint_f, w //2, h//2, 0.1, 100000),self.look_view
      )#pango.AxisDirection.AxisY))
    self.handler = pango.Handler3D(self.scam)
    
    # Interactive View in Window
    self.dcam = pango.CreateDisplay()
    self.dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -w/h)
    panel = pango.CreatePanel('ui')
    panel.SetBounds(0.0, 1.0, 0.0, 180/640.)

    self.axis = pango.Renderable()
    # print(dir(tree))
    # print(dir(pango.Axis()))
    self.axis.Add(pango.Axis())
    # def draw(view):
    #     view.Activate(self.scam)
    #     tree.Render()
    # self.dcam.SetDrawFunction(draw)
    # self.rename_id = pango.VarString('ui.rename id', "10")
    # self.rename_id_dir = pango.VarString('ui.rename id dir', "10")
    self.load_button =pango.VarFunc("ui.Open File",self.open_file_dialog)

    self.reset_view_button = pango.VarBool('ui.Reset View', value=False, toggle=False)
    self.show_grid = pango.VarBool('ui.Show Grid', value=True, toggle=True)
    self.show_id = pango.VarBool('ui.Show Id', value=False, toggle=False)

    self.show_g2o_id = pango.VarBool('ui.Show G2o Id', value=False, toggle=True)
    self.show_gtsam_id = pango.VarBool('ui.Show Gtsam Id', value=False, toggle=True)

    self.show_robot = pango.VarBool('ui.Show Robot', value=False, toggle=False)

    self.show_separator = pango.VarBool('ui.Show Separator', value=True, toggle=True)
    self.show_separator_only = pango.VarBool('ui.Only Show Separator', value=False, toggle=True)
   
    self.show_robot_1 = pango.VarBool('ui.Show Robot 1', value=True, toggle=True)
    self.show_robot_2 = pango.VarBool('ui.Show Robot 2', value=True, toggle=True)
    self.show_robot_3 = pango.VarBool('ui.Show Robot 3', value=True, toggle=True)
    self.show_robot_4 = pango.VarBool('ui.Show Robot 4', value=True, toggle=True)

    self.show_list = [self.show_robot_1,self.show_robot_2,self.show_robot_3,self.show_robot_4]



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

  
  def reload_graph(self,graph):
    self.graph = graph
    self.nodes = np.dot(graph.nodes, self.tform)
    self.edges = np.array(graph.edges)
    #keep default robot num
    # self.robot_num = robot_num
    self.nodes_keys = np.array(graph.nodes_keys)
    self.edges_key_pairs = np.array(graph.edges_key_pairs)
    self.multi_robot_tools = MultiRobotTools()
    self.separator_edges = self.graph.separator_edges 
    self.separator_nodes = np.dot(self.graph.separator_nodes,self.tform)
    self.partition_graph_np()

  def color_init(self):
    color_list = [[255,69,0],[255,215,0],[0,255,127],[0,191,255],[138,43,226]]
    self.color_list = color_list
    self.separator_edge_color = [244,164,96]
    self.separator_node_color = [255,250,205]

  def refresh(self):

    if pango.Pushed(self.reset_view_button):
      self.reset_view_callback()
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

    gl.glLineWidth(5)
    self.axis.Render()


    if self.show_grid.Get():
      gl.glLineWidth(1)
      self.drawPlane()
    
    if self.show_separator.Get():
      # draw separator
      if len(self.separator_nodes) >1:
        gl.glLineWidth(2)
        gl.glColor3f(self.separator_node_color[0]/255.0, self.separator_node_color[1]/255.0, self.separator_node_color[2]/255.0)
        pango.DrawCameras(self.separator_nodes,self.camera_size)
        
      if len(self.separator_edges) >1:
        gl.glLineWidth(3)
        gl.glColor3f(self.separator_edge_color[0]/255.0, self.separator_edge_color[1]/255.0, self.separator_edge_color[2]/255.0)
        pango.DrawLines(self.separator_edges[:,0,:-1, -1], self.separator_edges[:,1,:-1,-1])
    # pango.DrawText('Hello, world!', 20, 20)

    for robot_id in range(self.robot_num):
      edge_color = self.color_list[robot_id]
      nodes = self.nodes_dict[robot_id]
      edges = self.edges_dict[robot_id]
      if self.show_list[robot_id].Get() and not self.show_separator_only.Get():
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
    if(self.show_gtsam_id.Get() or self.show_g2o_id.Get()):
      self.draw_vertex_key()

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
                                and (self.multi_robot_tools.key2robot_id_g2o(key_pair[0])== self.multi_robot_tools.key2robot_id_g2o(key_pair[1]))
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

  def drawPlane(self,num_divs=30, div_size=5):
    # Plane parallel to x-z at origin with normal -y
    minx = -num_divs*div_size
    miny = -num_divs*div_size
    maxx = num_divs*div_size
    maxy = num_divs*div_size
    #gl.glLineWidth(2)
    #gl.glColor3f(0.7,0.7,1.0)
    gl.glColor3f(0.7,0.7,0.7)
    gl.glBegin(gl.GL_LINES)
    for n in range(2*num_divs):
        gl.glVertex3f(minx+div_size*n,miny,0)
        gl.glVertex3f(minx+div_size*n,maxy,0)
        gl.glVertex3f(minx,miny+div_size*n,0)
        gl.glVertex3f(maxx,miny+div_size*n,0)
    gl.glEnd()

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
    self.graph.update_key_position()
    self.update()
    self.partition_graph_np()

  def optimize_button_callback(self):
    self.graph.optimize(int(self.iterations.Get()))
    self.graph.update_key_position()
    self.update()
    self.partition_graph_np()
  
  def reset_view_callback(self):
    self.scam.SetModelViewMatrix(self.look_view)#pango.AxisDirection.AxisY))

  def draw_text(self,position,text):
    # pos_text = [0,0,0]
    # name = 'Hello'
    # gl.glDisable(gl.GL_LIGHTING)
    # gl.glColor3f(0.0, 0.0, 0.0)
    gl.glRasterPos3f(*position)
    glut.glutBitmapString(glut.GLUT_BITMAP_HELVETICA_12,
                          text.encode())
    # gl.glEnable(gl.GL_LIGHTING)
  
  def draw_vertex_key(self):
    for key, value in self.graph.key_node_dict.items():
      robot_id = self.multi_robot_tools.key2robot_id_g2o(key)
      if self.show_list[robot_id].Get():
        
        edge_color = self.color_list[robot_id]
        gl.glColor3f(edge_color[0]/255.0, edge_color[1]/255.0, edge_color[2]/255.0)

        edge_color = self.color_list[robot_id]

        if self.show_g2o_id.Get():
          self.draw_text(value,str(key))
        elif self.show_gtsam_id.Get():
          new_key =  self.multi_robot_tools.id_g2o2gtsam(key)
          self.draw_text(value,str(new_key))
    
  def open_file_dialog(self):
    # file_name, _ = QFileDialog.getOpenFileName(self.qt_main_window , "Open File", "../map", "Image Files (*.png *.jpg *.bmp)")
    # if file_name:
    #   print(file_name)
    name_list = self.file_name.split('/')
    base_dir = ''
    for each in name_list[0:-1]:
        base_dir+=each+'/'
    file_path = filedialog.askopenfilename(title="select input graph file",initialdir=base_dir)
    print(file_path)
    if is_file_renamed(file_path):
        pass
    else:
        g2o_tool = G2oTool()
        _,_ = g2o_tool.read(file_path)
        g2o_tool.rename_id()
        g2o_tool.reorder_id()
        str1,str2 = file_path.split('.')
        file_path = str1+'_renamed.g2o'
    graph = PoseGraph3D(verbose=True,use_transform=False)
    graph.load_file(file_path)
    self.reload_graph(graph)