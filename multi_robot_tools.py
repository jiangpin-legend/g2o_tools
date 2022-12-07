import os

from G2oReader import G2oReader

class Separator():
    def __init__(self) -> None:
        self.vertex = {}
        self.edge = {}
        
    def exisit(self,key_pair):
        for sep_key_pair in self.edge.keys:
            if(sep_key_pair[0] == key_pair[0] and sep_key_pair[1]==key_pair[1]):
                return True
            if(sep_key_pair[0] ==key_pair[1] and sep_key_pair[1]==key_pair[0]):
                return True
        return False
    
    def add(self,key_pair,vertex0,vertex1,edge):
        self.vertex[key_pair[0]] = vertex0
        self.vertex[key_pair[1]] = vertex1
        self.edge[key_pair] = edge


class MultiRobotTools():
    def __init__(self,data_dir,robot_num) -> None:
        self.data_dir = data_dir
        self.robot_num = robot_num
        self.vertex_dict = {}
        self.edge_dict = {}
        self.separator = Separator()
    
    def read_g2o(self):
        for robot_id in range(self.robot_num):
            file_name = os.path.join(self.data_dir,str(robot_id)+'.g2o')
            g2o_reader = G2oReader(file_name)
            vertex,edge = g2o_reader.read()
            self.vertex_dict[robot_id] = vertex
            self.edge_dict[robot_id] = edge
    
    def key2_robot_id(self,key):
        robot_id = int(key)//100000000000000000
        robot_id -= 69
        return robot_id

    def is_separator(self,key_pair):
        key0 = key_pair[0]
        key1 = key_pair[1]
        id0  = self.key2_robot_id(key0)
        id1  = self.key2_robot_id(key1)
        return id0 != id1


    def rename_gtsam_id(self):
        #rename the id in gtsam so that g2o can handle
        # (id in gtsam is too long)
        pass
    
    def aggregate_graph(self):
        #aggregate graph from each g2o file into one g2o file
        pass

    def partition_graph(self):
        #partition an aggregate graph
        pass
    
    def aggregate_separator(self):
        for each_robot in self.edge_dict.keys:
            edge_dict = self.edge_dict[each_robot]
            vertex_dict = self.vertex_dict[each_robot]
            for key_pair in edge_dict.keys:
                if(self.is_separator(key_pair)):
                    if(not self.separator.exsist(key_pair)):
                        vertex0 = vertex_dict[key_pair[0]]
                        vertex1 = vertex_dict[key_pair[1]]
                        edge = edge_dict[key_pair]
                        self.separator.add(key_pair,vertex0,vertex1,edge)


    def spread_separator(self):
        #spread separator among robot for dist-optimization
        pass
    