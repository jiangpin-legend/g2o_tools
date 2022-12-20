from g2o_tool import G2oTool

data_dir = '/home/jiangpin/dataset/new_4robots/'
robot_num = 4
g2o_tool = G2oTool()
for id in range(robot_num):
  g2o_file_name = data_dir+str(id)+'.g2o'
  _,_ = g2o_tool.read(g2o_file_name)
  g2o_tool.rename_id()
