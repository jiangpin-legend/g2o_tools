from multi_robot_tools import MultiRobotTools

if __name__ =="__main__":
  data_dir = "/home/jiangpin/dataset/2yuan_test/"
  num = 2
  multi_robot_tools = MultiRobotTools(data_dir,num)
  multi_robot_tools.read_g2o()
  multi_robot_tools.aggregate_graph()