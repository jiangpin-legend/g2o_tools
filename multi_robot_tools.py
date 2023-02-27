#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-

import os
import copy
import pickle
from math import sqrt

import g2o
import numpy as np
from sophus import SE3
from pyquaternion import Quaternion

from g2o_tool import G2oTool
from dijkstra import dijkstra,shortest_path

class Separator():
    def __init__(self,robot_num=0) -> None:
        self.vertex = {}
        self.edge = {}
        self.robot_num = robot_num

    def exists(self,key_pair):
        for sep_key_pair in self.edge.keys():
            if(sep_key_pair[0] == key_pair[0] and sep_key_pair[1]==key_pair[1]):
                return True
            if(sep_key_pair[0] ==key_pair[1] and sep_key_pair[1]==key_pair[0]):
                return True
        return False
    
    def add(self,key_pair,vertex0,vertex1,edge):
        self.vertex[key_pair[0]] = vertex0
        self.vertex[key_pair[1]] = vertex1
        self.edge[key_pair] = edge

    def add_vertex(self,key,vertex):
        pass

    def add_edge(self,key_pair,edge):
        self.edge[key_pair] = edge

    def sort_vertex(self):
        self.vertex = dict(sorted(self.vertex.items()))
        # print(self.vertex)
    
    def sort_vertex_by_id(self):
        self.robot_vertex = {}
        for id  in range(self.robot_num):
            self.robot_vertex[id] = {}
        # print(self.robot_vertex.keys())
        for item in self.vertex.items():
            key = item[0]
            value = item[1]
            id = self.key2robot_id_g2o(key)
            self.robot_vertex[id][key] = value
        
    def key2robot_id_g2o(self,key):
        robot_id = int(key)//1000000
        robot_id-=1
        return robot_id

class MultiRobotTools():
    def __init__(self,data_dir=None,robot_num=0) -> None:
        self.data_dir = data_dir
        self.robot_num = robot_num
        self.vertex_dict = {}
        self.edge_dict = {}
        #sum graph
        self.vertex_dict_sum = {}
        self.edge_dict_sum = {}
        #seprarator
        self.separator = Separator(robot_num)
        self.g2o_tool = G2oTool()
    
    def read_g2o(self):
        for robot_id in range(self.robot_num):
            file_name = os.path.join(self.data_dir,str(robot_id)+'.g2o')
            vertex,edge = self.g2o_tool.read(file_name)
            # print(edge)
            self.vertex_dict[robot_id] = vertex
            self.edge_dict[robot_id] = edge
    
    def read_g2o_renamed(self):
        self.optimizer = g2o.SparseOptimizer()
        self.edge_measure_dict = {}
        for robot_id in range(self.robot_num):
            file_name = os.path.join(self.data_dir,str(robot_id)+'_renamed.g2o')
            vertex,edge = self.g2o_tool.read(file_name)
            # print(edge)
            self.vertex_dict[robot_id] = vertex
            self.edge_dict[robot_id] = edge
            measure_dict = {}
            for edge in self.optimizer.edges():
                key_pair = (edge.vertices()[0].id(),edge.vertices()[1].id())
                measure_dict[key_pair] = edge.measurement().matrix()
            self.edge_measure_dict[robot_id] = measure_dict

    def rename_gtsam_id(self):

        edge_dict_copy = copy.deepcopy(self.edge_dict)
        vertex_dict_copy = copy.deepcopy(self.vertex_dict)
        vertex_rename_sum = {}
        edge_rename_sum = {}
        for robot_id in range(self.robot_num):
            vertex_dict = vertex_dict_copy[robot_id]
            edge_dict = edge_dict_copy[robot_id]
            # print(robot_id)
            # print(vertex_dict.keys())
            # print(edge_dict.keys())
            new_vertex_dict = {}
            for key in vertex_dict.keys():
                newkey = self.id_gtsam2g2o(key)
                new_vertex_dict[newkey] = vertex_dict[key]

            # vertex_dict[newkey] = vertex_dict.pop(key)
            new_edge_dict = {}

            for key_pair in edge_dict.keys():
                new_key0 = self.id_gtsam2g2o(key_pair[0])
                new_key1 = self.id_gtsam2g2o(key_pair[1])
                new_edge_dict[(new_key0,new_key1)] = edge_dict[key_pair]
                # edge_dict[(new_key0,new_key1)] = edge_dict.pop(key_pair)
            vertex_rename_sum.update(new_vertex_dict)
            edge_rename_sum.update(new_edge_dict)
            file_name = os.path.join(self.data_dir,str(robot_id)+'_renamed.g2o')
            # file_name = os.path.join(self.data_dir,str(robot_id)+'.g2o')
            self.g2o_tool.write_dict(file_name,new_vertex_dict,new_edge_dict)
        self.aggregate_graph()
        file_name = os.path.join(self.data_dir,'full_graph_renamed.g2o')
        # file_name = os.path.join(self.data_dir,'full_graph.g2o')
        self.g2o_tool.write_dict(file_name,vertex_rename_sum,edge_rename_sum)


    def is_separator(self,key_pair):
        key0 = key_pair[0]
        key1 = key_pair[1]
        id0  = self.key2robot_id(key0)
        id1  = self.key2robot_id(key1)
        return id0 != id1

    def is_separator_g2o(self,key_pair):
        key0 = key_pair[0]
        key1 = key_pair[1]
        id0  = self.key2robot_id_g2o(key0)
        id1  = self.key2robot_id_g2o(key1)
        return id0!=id1

    def exist_odometry(self,key_pair,edge_dict):
        key0 = int(key_pair[0])
        key1 = int(key_pair[1])

        assert(int(key0)<int(key1))
        assert(self.key2robot_id_g2o(key0)==self.key2robot_id_g2o(key1))

        for i in range(int(key1)-int(key0)-1):
            temp_key0 = str(key0+i)
            temp_key1 = str(key0+i+1)
            key_pair = (temp_key0,temp_key1)
            if key_pair not in edge_dict.keys():
                return False
        return True 

    def key2robot_id(self,key):
        robot_id = int(key)//100000000000000000
        robot_id -= 69
        return robot_id

    def key2robot_id_g2o(self,key):
        robot_id = int(key)//1000000
        robot_id-=1
        return robot_id

    def id_gtsam2g2o(self,key):
        #rename the id in gtsam so that g2o can handle
        # (id in gtsam is too long)
        key = int(key)
        robot_id = self.key2robot_id(key)
        result =key//1000000
        #10^6
        robot_id+=1
        new_key = robot_id*1000000+key-result*1000000
        return str(new_key)
    def is_renamed(file_name):
    #drop .g2o
        words = file_name.split('.')[0]
        words = words.split('_')
        if 'renamed' in words:
            return True
        else:
            return False
    
    def aggregate_graph(self):
        #aggregate graph from each g2o file into one g2o file
        for robot_id in range(self.robot_num):
            vertex_dict = self.vertex_dict[robot_id]
            edge_dict = self.edge_dict[robot_id]

            self.vertex_dict_sum.update(vertex_dict)
            self.edge_dict_sum.update(edge_dict)
        file_name = os.path.join(self.data_dir,'full_graph'+'.g2o')
        self.g2o_tool.write_dict(file_name,self.vertex_dict_sum,self.edge_dict_sum)


    def partition_graph(self,file_name):
        vertex,edge = self.g2o_tool.read(file_name)
        base_name_list = file_name.split('/')
        base_name = ''
        for i in range(len(base_name_list)-1):
            base_name =base_name+base_name_list[i]+'/'
        vertex_dict = {}
        edge_dict = {}
        
        for robot_id in range(self.robot_num):
            vertex_dict[robot_id] = {}
            edge_dict[robot_id] = {}
            for key in vertex.keys():
                if(self.key2robot_id(key)==robot_id):
                    vertex_dict[robot_id].update({key:vertex[key]})
            for key_pair in edge.keys():
                if(self.key2robot_id(key_pair[0])==robot_id or self.key2robot_id(key_pair[1])==robot_id):
                    edge_dict[robot_id].update({key_pair:edge[key_pair]})
            self.g2o_tool.write_dict(base_name+str(robot_id)+'.g2o',vertex_dict[robot_id],edge_dict[robot_id])

    def partition_graph_g2o(self,file_name):
            vertex,edge = self.g2o_tool.read(file_name)
            base_name_list = file_name.split('/')
            base_name = ''
            for i in range(len(base_name_list)-1):
                base_name =base_name+base_name_list[i]+'/'
            vertex_dict = {}
            edge_dict = {}
            
            for robot_id in range(self.robot_num):
                vertex_dict[robot_id] = {}
                edge_dict[robot_id] = {}
                for key in vertex.keys():
                    if(self.key2robot_id_g2o(key)==robot_id):
                        vertex_dict[robot_id].update({key:vertex[key]})
                for key_pair in edge.keys():
                    if(self.key2robot_id_g2o(key_pair[0])==robot_id or self.key2robot_id_g2o(key_pair[1])==robot_id):
                        edge_dict[robot_id].update({key_pair:edge[key_pair]})
                self.g2o_tool.write_dict(base_name+str(robot_id)+'.g2o',vertex_dict[robot_id],edge_dict[robot_id])

    def aggregate_separator(self):
        for each_robot in self.edge_dict.keys():
            edge_dict = self.edge_dict[each_robot]
            for key_pair in edge_dict.keys():
                if(self.is_separator(key_pair)):
                    if(not self.separator.exists(key_pair)):
                        id0 = self.key2robot_id(key_pair[0])
                        id1 = self.key2robot_id(key_pair[1])
                        
                        vertex0 = self.vertex_dict[id0][key_pair[0]]
                        vertex1 = self.vertex_dict[id1][key_pair[1]]
                        edge = edge_dict[key_pair]
                        self.separator.add(key_pair,vertex0,vertex1,edge)
        file_name = os.path.join(self.data_dir,'separator'+'.g2o')
        self.g2o_tool.write_dict(file_name,self.separator.vertex,self.separator.edge)
    
    def aggregate_separator_g2o(self):
        for each_robot in self.edge_dict.keys():
            edge_dict = self.edge_dict[each_robot]
            for key_pair in edge_dict.keys():
                print(key_pair)
                if(self.is_separator_g2o(key_pair)):
                    if(not self.separator.exists(key_pair)):
                        id0 = self.key2robot_id_g2o(key_pair[0])
                        id1 = self.key2robot_id_g2o(key_pair[1])
                        
                        vertex0 = self.vertex_dict[id0][key_pair[0]]
                        vertex1 = self.vertex_dict[id1][key_pair[1]]
                        edge = edge_dict[key_pair]
                        self.separator.add(key_pair,vertex0,vertex1,edge)
        #add edge between separator
        # for key_0 in self.separator.vertex.keys():
        #     for key_1 in self.separator.vertex.keys():
        #         if(key_0!=key_1):
        #             key_pair = (key_0,key_1)
        #             if key_pair in edge_dict.keys():
        #                 self.separator.add_edge(key_pair,edge_dict[key_pair])
        self.separator.sort_vertex()
        self.separator.sort_vertex_by_id()
        file_name = os.path.join(self.data_dir,'separator'+'.g2o')
        self.g2o_tool.write_dict(file_name,self.separator.vertex,self.separator.edge)

    def spread_separator(self):
        #spread separator among robot for dist-optimization
        self.aggregate_separator_g2o()
        edge_dict_copy = copy.deepcopy(self.edge_dict)
        vertex_dict_copy = copy.deepcopy(self.vertex_dict)

        edge_dict_sum = copy.deepcopy(self.edge_dict_sum)
        vertex_dict_sum = copy.deepcopy(self.vertex_dict_sum)

        for robot_id in edge_dict_copy.keys():
            edge_dict = edge_dict_copy[robot_id]
            vertex_dict = vertex_dict_copy[robot_id]
            # print(self.separator.edge.keys())
            edge_dict.update(self.separator.edge)
            vertex_dict.update(self.separator.vertex)
            self.connect_separator_single(robot_id,edge_dict)
            edge_dict_sum.update(edge_dict)
            file_name = os.path.join(self.data_dir,str(robot_id)+'_separator'+'.g2o')
            # file_name = os.path.join(self.data_dir,str(robot_id)+'.g2o')
            
            self.g2o_tool.write_dict(file_name,vertex_dict,edge_dict)

        
        edge_dict_sum.update(self.separator.edge)
        vertex_dict_sum.update(self.separator.vertex)
        file_name = os.path.join(self.data_dir,'full_graph_separator'+'.g2o')
        self.g2o_tool.write_dict(file_name,vertex_dict_sum,edge_dict_sum)


    # def make_graph(self):
    #     graph = {}
    #     for each_robot in self.edge_dict.keys():
    #         edge_dict = self.edge_dict[each_robot]
    #         for key_pair in edge_dict.keys():
    #             if key_pair[0] in graph.keys():
    #                 graph[key_pair[0]][key_pair[1]] = 1
    #             else:
    #                 graph[key_pair[0]] = {key_pair[1]:1}

    #             if key_pair[1] in graph.keys():
    #                 graph[key_pair[1]][key_pair[0]] = 1
    #             else:
    #                 graph[key_pair[1]] = {key_pair[0]:1}
    #     self.graph = graph
    #     # print(graph)
    
    def make_graph(self):
        graph_robot = {}
        for each_robot in self.edge_dict.keys():
            graph = {}
            edge_dict = self.edge_dict[each_robot]
            for key_pair in edge_dict.keys():
                if key_pair[0] in graph.keys():
                    graph[key_pair[0]][key_pair[1]] = 1
                else:
                    graph[key_pair[0]] = {key_pair[1]:1}

                if key_pair[1] in graph.keys():
                    graph[key_pair[1]][key_pair[0]] = 1
                else:
                    graph[key_pair[1]] = {key_pair[0]:1}
            graph_robot[each_robot] = graph
        self.graph_robot = graph_robot
        # print(graph)
    
    def connect_separator_single(self,robot_id,edge_dict):        
        vertex = self.separator.vertex
        key_list = list(vertex.keys())
        # print(key_list)
        add_odom_count = 0
        for i in range(len(key_list)-1):
            key0 = key_list[i]
            key1 = key_list[i+1]
            id0 = self.key2robot_id_g2o(key0)
            id1 = self.key2robot_id_g2o(key1)

            if(id0==id1):
                if(id0!=robot_id):
                    # if not (self.exist_odometry((key0,key1),self.edge_dict[robot_id])):
                    m = np.matrix([[1,0,0,0],
                                    [0,1,0,0],
                                    [0,0,1,0],
                                    [0,0,0,1]])
                    for key in range(int(key0),int(key1)):
                        measurement = self.edge_measure_dict[(key,key+1)]
                        # m = np.dot(measurement,m)
                        m = np.dot(m,measurement)


                    #add separator odometry
                    key_pair = (key0,key1)
                    q = Quaternion(matrix=m)
                    quaternion = [0,0,0,0]
                    quaternion[0] = q.x
                    quaternion[1] = q.y
                    quaternion[2] = q.z
                    quaternion[3] = q.w

                    # quaternion= self.matrix_to_quaternion(m[0:3,0:3])
                    translation =[0,0,0]
                    translation[0] = m[0,3]
                    translation[1] = m[1,3]
                    translation[2] = m[2,3]
                    measurement = translation+quaternion
                    measurement = [str(each) for each in measurement]
                    info_mat = '1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1\n'
                    info_mat = info_mat.split(' ')
                    edge_dict[key_pair] = measurement+info_mat
                    # print('add odometry')
                    print(key0,key1)
                    add_odom_count +=1
        print('add %d odom edge to graph%d'%(add_odom_count,robot_id))
      
                
                # self.separator.add_edge(key_pair,measurement+info_mat)
    
    def connect_separator(self):
        path_dict = {}
        # for key1 in self.separator.vertex.keys():
        #     for key2 in self.separator.vertex.keys():
        #         if(key1!=key2):
        #             if( not((key2,key1) in path_dict.keys()) ):
        #                 path = shortest_path(self.graph,key1,key2)
        #                 path_dict[(key1,key2)] = path
        # with open('./path.pkl','wb') as f:
        #     pickle.dump(path_dict,file=f)

        for robot_id in self.graph_robot.keys():
            vertex = self.separator.robot_vertex[robot_id]
            key_list = list(vertex.keys())
            # print(key_list)
            for i in range(len(key_list)-1):
                key0 = key_list[i]
                key1 = key_list[i+1]
                id0 = self.key2robot_id_g2o(key0)
                id1 = self.key2robot_id_g2o(key1)

                if(id0==id1):
                    graph =self.graph_robot[id0]
                    if((int(key1)-int(key0))>0):
                        path = shortest_path(graph,key0,key1)
                        print(key0,key1,path)
                        path_dict[(key0,key1)] = path
        
        with open('./path.pkl','wb') as f:
            pickle.dump(path_dict,file=f)

        
        # with open('./path.pkl','rb') as f:
        #     path_dict = pickle.load(f)

        self.measurement_dict = {}
        # print(self.edge_measure_dict.keys())
        for key_pair,path in zip(path_dict.keys(),path_dict.values()):
            m = np.matrix([[1,0,0,0],
                           [0,1,0,0],
                           [0,0,1,0],
                           [0,0,0,1]])
            if(len(path)>0):
                for i in range(len(path)-1):
                    key0 = path[i]
                    key1 = path[i+1]
                    try:
                        measurement = self.edge_measure_dict[(int(key0),int(key1))]
                        m = np.dot(measurement,m)
                        # print('normal')
                        
                        continue
                    except:
                        print('exception')
                        measurement = self.edge_measure_dict[(int(key1),int(key0))]
                        m_inv = SE3(measurement).inverse().matrix()
                        m = np.matmul(m_inv,m)

              
                quaternion= self.matrix_to_quaternion(m[0:3,0:3])
                translation =[0,0,0]
                translation[0] = m[0,3]
                translation[1] = m[1,3]
                translation[2] = m[2,3]


                # print(type(translation))
                # print(translation)

                measurement = translation+quaternion
                measurement = [str(each) for each in measurement]

                info_mat = '1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1\n'
                info_mat = info_mat.split(' ')
                self.measurement_dict[key_pair] = measurement+info_mat
                self.separator.add_edge(key_pair,measurement+info_mat)

           
            
        #print(self.measurement_dict)
                
    def load_measurement(self):
        self.optimizer = g2o.SparseOptimizer()
        file_name = os.path.join(self.data_dir,'full_graph'+'.g2o')
        self.optimizer.load(file_name)
        self.edge_measure_dict = {}
        for edge in self.optimizer.edges():
            key_pair = (edge.vertices()[0].id(),edge.vertices()[1].id())
            self.edge_measure_dict[key_pair] = edge.measurement().matrix()

    def matrix_to_quaternion(self,R):
        trace = R[0,0]+R[1,1]+R[2,2]
        Q = [0,0,0,0]
        if trace >0:
            s = sqrt(trace+1.0)
            Q[3] = (s*0.5)
            s = 0.5/s
            Q[0] = (R[2,1]-R[1,2])*s
            Q[1] = (R[0,2]-R[2,0])*s
            Q[2] = (R[1,0]-R[0,1])*s
        else:
            b = 2 if R[0,0]<R[2,2] else 0
            a = 2 if R[1,1]<R[2,2] else 1
            i = a if R[0,0]<R[1,1] else b
            j = (i+1)%3
            k = (i+2)%3

            s = sqrt(R[i,j]-R[j,j]-R[k,k]+1)
            Q[i] = s*0.5
            s = 0.5/s

            Q[3] = (R[k,j]-R[j,k])*s
            Q[j] = (R[j,i]+R[i,j])*s
            Q[k] = (R[k,i]+R[i,k])*s
        return Q

    def central_data_pipeline(self):
        pass
    def graph_info(self):
        print("----Total info---")
        print("total vertex num:"+str(len(self.vertex_dict_sum)))
        print("total edge num:"+str(len(self.edge_dict_sum)))
        print("---Separator info---")
        print("separator vertex num:"+str(len(self.separator.vertex)))
        print("separator edge num:"+str(len(self.separator.edge)))



if __name__ == '__main__':
    # data_dir = '/home/jiangpin/dataset/test_4robots'
    data_dir = "/home/jiangpin/dataset/new_4robots/"
    data_dir = "/home/jiangpin/dataset/3dog/"
    # data_dir = "/home/jiangpin/dataset/2yuan_new/"
    # data_dir = "/home/jiangpin/dataset/2yuan_test/"


    # data_dir = "/home/jiangpin/dataset/simulation/example_4robots/"

    num = 3
    # multi_robot_tools = MultiRobotTools(data_dir,num)
    # multi_robot_tools.read_g2o()
    # multi_robot_tools.rename_gtsam_id()
    # del multi_robot_tools
    multi_robot_tools = MultiRobotTools(data_dir,num)
    multi_robot_tools.read_g2o_renamed()
    multi_robot_tools.aggregate_graph()
    multi_robot_tools.aggregate_separator_g2o()
    multi_robot_tools.make_graph()
    multi_robot_tools.load_measurement()
    # multi_robot_tools.connect_separator()
    multi_robot_tools.spread_separator()
    multi_robot_tools.graph_info()
