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