from g2o_tool import G2oTool

g2o_file_name = '/home/jiangpin/dataset/simulation/example_4robots/fullGraph_optimized.g2o'
g2o_file_name = '/home/jiangpin/dataset/3dog/fullGraph_optimized.g2o'
g2o_file_name = '/home/jiangpin/dataset/2yuan/readFullGraph.g2o'
g2o_file_name = '/home/jiangpin/graph/graph/readFullGraph.g2o'
g2o_file_name = '/home/jiangpin/graph/graph/fullGraph.g2o'
g2o_file_name = '/home/jiangpin/graph/0221/graph/readFullGraph.g2o'
g2o_file_name = '/home/jiangpin/dataset/graph/readFullGraph.g2o'

# g2o_file_name = '/home/jiangpin/graph/0221/graph/fullGraph.g2o'




g2o_tool = G2oTool()

_,_ = g2o_tool.read(g2o_file_name)
g2o_tool.rename_id()
