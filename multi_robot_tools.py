import os

from G2oReader import G2oReader

class MultiRobotTools():
    def __init__(self,data_dir,robot_num) -> None:
        self.data_dir = data_dir
        self.robot_num = robot_num
        self.vertex_dict = {}
        self.edge_dict = {}
    
    def read_g2o(self):
        for robot_id in range(self.robot_num):
            file_name = os.path.join(self.data_dir,str(robot_id)+'.g2o')
            g2o_reader = G2oReader(file_name)
            vertex,edge = g2o_reader.read()
            self.vertex_dict[robot_id] = vertex
            self.edge_dict[robot_id] = edge


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

    def spread_separator(self):
        #spread separator among robot for dist-optimization
        pass
