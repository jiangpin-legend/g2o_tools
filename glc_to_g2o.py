import pickle
from g2o_tool import G2oTool
import math
import re

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    
    return [x, y, z, w]

class GlcConverter(G2oTool):
    def __init__(self) -> None:
        super().__init__()
    
    def read(self,file_name):
        self.file_name = file_name
        vertex = {}
        edge = {}
        name_list = self.file_name.split('/')
        self.base_dir = ''
        for each in name_list[0:-1]:
            self.base_dir+=each+'/'
        self.base_name = self.file_name.split('.')
        with open(file_name,'r') as g2o_file:
            for each_line in g2o_file:
                try:
                    line = re.sub(r'\s+',' ',each_line)
                    g2o__line = line.split(' ')
                    identifier= g2o__line[0]
                    g2o__line.append('\n')
                    if identifier == 'VERTEX_SE3:QUAT':
                        vertex[g2o__line[1]] = (g2o__line[2:])
                    elif identifier=='GLC_EDGE':
                        if(len(g2o__line)>10):
                            rpy = []
                            for i in range(6,9):
                                rpy.append(float(g2o__line[i]))
                            quat = euler_to_quaternion(rpy[0],rpy[1],rpy[2])

                            new_g2o_line = []
                            #rpy to quaternion
                            for i in range(3,6):
                                new_g2o_line.append(g2o__line[i])
                            #     print(g2o__line[i])
                            # print((g2o__line[0:4]))
                            # print("====")
                            for q in quat:
                                q = round(q,6)
                                new_g2o_line.append(str(q))
                            for i in range(9,30):
                                new_g2o_line.append(g2o__line[i])
                            new_g2o_line.append('\n')
                            edge[(g2o__line[1],g2o__line[2])] = (new_g2o_line)


                except ValueError:
                    pass
        self.vertex = vertex
        self.edge = edge
        return vertex,edge
    
    def process(self):
        file_name = self.base_name[0]+'_processed.g2o'
        self.write_dict(file_name,self.vertex,self.edge)

if __name__ == '__main__':
    glc_to_g2o = GlcConverter()
    # vertex,edge = g2o_tool.read('/home/jiangpin/dataset/example_4robots/0.g2o')
    # vertex,edge = g2o_tool.read('/home/jiangpin/dataset/new_4robots/0.g2o')
    _,_ = glc_to_g2o.read('/home/jiangpin/projects/Multi-Robot-SLAM/SparsifyPoseGraph/datasets/online/2/s3e/glc_dense_g/marginal_1610.g2o')
    glc_to_g2o.process()
    # g2o_tool.rename_id()
    # g2o_tool.indentify_orentation()
    # vertex,edge = g2o_tool.read('/home/jiangpin/dataset/example_4robots/0.g2o')

    # edge_keys = g2o_tool.read_edge_keys('/home/jiangpin/dataset/example_4robots/0.g2o')

    # print(vertex)
    # print()
    # print(edge_keys)