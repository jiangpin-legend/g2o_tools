#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-

class G2oTool:
    def __init__(self) -> None:
        self.vertex = {}
        self.edge = {}
    
    def read(self,file_name):
        self.file_name = file_name
        vertex = {}
        edge = {}
        with open(file_name,'r') as g2o_file:
            for each_line in g2o_file:
                try:
                    g2o__line = each_line.split(' ')
                    identifier= g2o__line[0]
                    if identifier == 'VERTEX_SE3:QUAT':
                        vertex[g2o__line[1]] = (g2o__line[2:])
                    elif identifier=='EDGE_SE3:QUAT':
                        edge[(g2o__line[1],g2o__line[2])] = (g2o__line[3:])
                except ValueError:
                    pass
        self.vertex = vertex
        self.edge = edge
        return vertex,edge
    
    def read_edge_keys(self,file_name):
        edge_keys = []
        with open(file_name,'r') as g2o_file:
            for each_line in g2o_file:
                try:
                    g2o__line = each_line.split('    ')
                    identifier= g2o__line[0]
                    if identifier == 'VERTEX_SE3:QUAT':
                        pass
                    elif identifier=='EDGE_SE3:QUAT':
                        edge_keys.append((int(g2o__line[1]),int(g2o__line[2])))
                except ValueError:
                    pass
        return edge_keys

    def write(self,file_name,vertex_lines,edge_lines):
        with open(file_name,'w') as g2o_file:
            g2o_file.writelines(vertex_lines)
            g2o_file.writelines(edge_lines) 

    def write_dict(self,file_name,vertex_dict,edge_dict):
        vertex_lines = self.vertex_dict2lines(vertex_dict)
        edge_lines = self.edge_dict2lines(edge_dict)
        self.write(file_name,vertex_lines,edge_lines)

    def vertex_dict2list(self,vertex_dict):
        vertex_list = []
        for key,data in zip(vertex_dict.keys(),vertex_dict.values()):
            vertex_list.append(key)
            for each in data:
                vertex_list.append(each)
        return vertex_list

    def edge_dict2list(self,edge_dict):
        edge_list = []
        for key_pair,data in zip(edge_dict.keys(),edge_dict.values()):
            edge_list.append(key_pair[0])
            edge_list.append(key_pair[1])
            for each in data:
                edge_list.append(each)
        return edge_list
    
    def vertex_dict2lines(self,vertex_dict):
        vertex_lines = []
        for key,data in zip(vertex_dict.keys(),vertex_dict.values()):
            line = 'VERTEX_SE3:QUAT '
            line =line+key
            for each in data:
                line = line+' '+each
            # line += '\n'
            vertex_lines.append(line)
        return vertex_lines

    def edge_dict2lines(self,edge_dict):
        edge_lines = []
        for key_pair,data in zip(edge_dict.keys(),edge_dict.values()):
            edge_line = 'EDGE_SE3:QUAT '
            
            edge_line += key_pair[0]+' '+key_pair[1]
            for each in data:
                edge_line = edge_line+' '+each
            # edge_line += '\n'
            edge_lines.append(edge_line)
        return edge_lines

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

    def id_g2o2gtsam(self,key):
        gtsam_id = [6989586621679000000,7061644215716930000,7133701809754860000]
        robot_id = self.key2robot_id_g2o(key)
        result = gtsam_id[robot_id]+key%10000
        return result

    def rename_id(self):
        renamed_vertex = {}
        for key in self.vertex.keys():
            new_key = self.id_gtsam2g2o(key)
            renamed_vertex[new_key] = self.vertex[key]

        renamed_edge = {}
        for key_pair in self.edge.keys():
            new_key0 = self.id_gtsam2g2o(key_pair[0])
            new_key1 = self.id_gtsam2g2o(key_pair[1])
            new_key_pair = (new_key0,new_key1)
            renamed_edge[new_key_pair] = self.edge[key_pair]
        base_name = self.file_name.split('.')
        # print(base_name)
        file_name = base_name[0]+'_renamed.g2o'
        self.write_dict(file_name,renamed_vertex,renamed_edge)
        
    def indentify_orentation(self):
        for key in self.edge.keys():
            value = self.edge[key]
            value[3] = '0'
            value[4] = '0'
            value[5] = '0'
            value[6] = '1'

            self.edge[key] = value
            
        base_name = self.file_name.split('.')
        file_name = base_name[0]+'_identity.g2o'
        self.write_dict(file_name,self.vertex,self.edge)
        

if __name__ == '__main__':
    g2o_tool = G2oTool()
    # vertex,edge = g2o_tool.read('/home/jiangpin/dataset/example_4robots/0.g2o')
    # vertex,edge = g2o_tool.read('/home/jiangpin/dataset/new_4robots/0.g2o')
    _,_ = g2o_tool.read('/home/jiangpin/dataset/2yuan_test/full_graph.g2o')
    # g2o_tool.rename_id()
    # g2o_tool.indentify_orentation()
    # vertex,edge = g2o_tool.read('/home/jiangpin/dataset/example_4robots/0.g2o')

    # edge_keys = g2o_tool.read_edge_keys('/home/jiangpin/dataset/example_4robots/0.g2o')

    # print(vertex)
    # print()
    # print(edge_keys)