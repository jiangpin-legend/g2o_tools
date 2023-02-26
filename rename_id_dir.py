import os

from g2o_tool import G2oTool

def is_renamed(file_name):
  #drop .g2o
  words = file_name.split('.')[0]
  words = words.split('_')
  if 'renamed' in words:
    return True
  else:
    return False

data_dir = '/home/jiangpin/dataset/new_4robots/'
data_dir = '/home/jiangpin/dataset/2yuan/'
data_dir = '/home/jiangpin/dataset/2yuan_new/'


file_list = os.listdir(data_dir)
robot_num = 2
g2o_tool = G2oTool()

# # for id in range(robot_num):
# #   g2o_file_name = data_dir+str(id)+'.g2o'
# #   _,_ = g2o_tool.read(g2o_file_name)
# #   g2o_tool.rename_id()

for file in file_list:
  
  g2o_file_name = data_dir+file
  if not is_renamed(g2o_file_name):
    _,_ = g2o_tool.read(g2o_file_name)
    g2o_tool.rename_id()