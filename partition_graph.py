from multi_robot_tools import MultiRobotTools

if __name__ =="__main__":
  # filename = "/home/jiangpin/dataset/2yuan/readFullGraph.g2o"
  # data_dir = "/home/jiangpin/dataset/2yuan/"
  filename = "/home/jiangpin/dataset/graph/readFullGraph.g2o"
  data_dir = "/home/jiangpin/dataset/graph/"

  filename = "/home/jiangpin/dataset/2yuan_new/full_graph.g2o"
  data_dir = "/home/jiangpin/dataset/2yuan_new/"
  num = 2
  
  filename = "/home/jiangpin/dataset/2yuan_test/full_graph.g2o"
  data_dir = "/home/jiangpin/dataset/2yuan_test/"


  multi_robot_tools = MultiRobotTools(data_dir,num)
  # multi_robot_tools.read_g2o()
  multi_robot_tools.partition_graph_g2o(filename)


