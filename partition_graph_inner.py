# from multi_robot_tools import MultiRobotTools
from g2o_tool import G2oTool

if __name__ =="__main__":
  

  # g2o_file_name = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/S3E_clollege_025/full_graph.g2o"
  # g2o_file_name = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/3_robot_1116/full_graph.g2o"
  g2o_file_name = "/home/jiangpin/dataset/s3e/full_graph.g2o"
  

  g2o_tool = G2oTool()

  _,_ = g2o_tool.read(g2o_file_name)
  g2o_tool.seperate_inner()


