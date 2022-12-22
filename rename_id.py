from g2o_tool import G2oTool

g2o_file_name = '/home/jiangpin/dataset/simulation/example_4robots/fullGraph_optimized.g2o'


g2o_tool = G2oTool()

_,_ = g2o_tool.read(g2o_file_name)
g2o_tool.rename_id()
