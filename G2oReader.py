#!/usr/bin/python3.6
# -*- coding: UTF-8 -*-

def edge_dict2list():
    pass
def vertex_dict2list():
    pass

class G2oReader:
    def __init__(self) -> None:
        self.vertex = {}
        self.edge = {}
    
    def read(self,file_name):
        self.file_name = file_name
        self.data = open(file_name,'r')
        for each_line in self.data:
            try:
                g2o__line = each_line.split(' ')
                identifier= g2o__line[0]
                if identifier == 'VERTEX_SE3:QUAT':
                    self.vertex[g2o__line[1]] = (g2o__line[2:-1])
                elif identifier=='EDGE_SE3:QUAT':
                    self.edge[(g2o__line[1],g2o__line[2])] = (g2o__line[3:-1])
            except ValueError:
                pass
        self.data.close()
        return self.vertex,self.edge
  

if __name__ == '__main__':
    g2o_reader = G2oReader()
    vertex,edge = g2o_reader.read('/home/jiangpin/dataset/example_4robots/0.g2o')
    print(vertex)
    print()
    print(edge)