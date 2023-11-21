
import sys
import pickle

import g2o
import numpy as np
import re
# from sophus import *

from viewer import Viewer3D
from multi_viewer import MultiViewer3D
from multi_robot_tools import MultiRobotTools
from g2o_tool import G2oTool
from posegraph import PoseGraph3D

def is_file_renamed(filename):
  sub_string = 'renamed'
  pattern = re.compile(sub_string)
  if pattern.search(filename):
    return True
  else:
    return False
    
if len(sys.argv) > 1:
    gfile = str(sys.argv[1])
else:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
    # gfile = "/home/jiangpin/dataset/example_4robots/3_renamed.g2o"
    # gfile = "/home/jiangpin/dataset/example_4robots/full_graph_renamed.g2o"
    # gfile = "/home/jiangpin/dataset/simulation/example_4robots/full_graph_renamed.g2o"
    # gfile = "/home/jiangpin/dataset/simulation/example_4robots/fullGraph_optimized_renamed.g2o"
    # gfile = "/home/jiangpin/dataset/new_4robots/0_separator_optimized.g2o"
    # gfile = "/home/jiangpin/dataset/new_4robots/1_separator_optimized.g2o"
    # gfile = "/home/jiangpin/dataset/new_4robots/2_separator_optimized.g2o"
    # gfile = "/home/jiangpin/dataset/new_4robots/3_separator_optimized.g2o"
    # gfile = "/home/jiangpin/dataset/new_4robots/separator.g2o"

    # gfile = "/home/jiangpin/dataset/new_4robots/full_graph_optimized.g2o"
    # gfile = "/home/jiangpin/dataset/3dog/full_graph_renamed.g2o"
    # gfile = '/home/jiangpin/dataset/3dog/0_separator_optimized.g2o'
    # gfile = '/home/jiangpin/dataset/3dog/1_separator_optimized.g2o'
    # gfile = '/home/jiangpin/dataset/3dog/2_separator_optimized.g2o'
    # gfile = '/home/jiangpin/dataset/3dog/fullGraph_optimized_renamed.g2o'
    # gfile = '/home/jiangpin/dataset/3dog/full_graph.g2o'


    
    # gfile = '/home/jiangpin/dataset/3dog/full_graph.g2o'

    # gfile = '/home/jiangpin/dataset/3dog/separator.g2o'
    # gfile = '/home/jiangpin/dataset/3dog/0_separator.g2o'
    # gfile = '/home/jiangpin/dataset/3dog/1_separator.g2o'
    # gfile = '/home/jiangpin/dataset/3dog/2_separator.g2o'

    gfile = '/home/jiangpin/dataset/3dog/0_separator_optimized.g2o'
    gfile = '/home/jiangpin/dataset/3dog/1_separator_optimized.g2o'
    gfile = '/home/jiangpin/dataset/3dog/2_separator_optimized.g2o'



    # gfile = "/home/jiangpin/dataset/3dog/full_graph_renamed_optimized.g2o"

    # gfile = "/home/jiangpin/dataset/new_4robots/separator.g2o" 
    # gfile = "/home/jiangpin/dataset/new_4robots/two_stage_centralized.g2o"

    # gfile = '/home/jiangpin/dataset/2yuan/readFullGraph_renamed.g2o'
  

    # gfile = "./data/sphere2500.g2o"
    # gfile = '/home/jiangpin/graph/graph/readFullGraph_renamed.g2o'
    # gfile = '/home/jiangpin/graph/0221/graph/readFullGraph_renamed.g2o'

    # gfile = '/home/jiangpin/graph/0221/graph/optimized.g2o'
    # gfile = '/home/jiangpin/dataset/2yuan_new/full_graph_renamed.g2o'

    # gfile = '/home/jiangpin/dataset/2yuan_new/separator.g2o'
    # gfile = '/home/jiangpin/dataset/2yuan_new/0_separator.g2o'
    # gfile = '/home/jiangpin/dataset/2yuan_new/1_separator.g2o'
    # gfile = '/home/jiangpin/dataset/2yuan_new/0_separator_optimized.g2o'
    # gfile = '/home/jiangpin/dataset/2yuan_new/0_separator_optimized_renamed.g2o'

    # gfile = '/home/jiangpin/dataset/2yuan_new/1_separator_optimized.g2o'
    # gfile = '/home/jiangpin/dataset/2yuan_new/full_graph.g2o'
    # gfile = '/home/jiangpin/dataset/2yuan_new/full_graph_separator.g2o'

    gfile = '/home/jiangpin/dataset/2yuan_test/full_graph.g2o'
    gfile = '/home/jiangpin/dataset/2yuan_test/full_graph.g2o'
    gfile = '/home/nuc/github/lusha/Multi-robot-SLAM/MR_SLAM/Mapping/src/global_manager/log/optimized_renamed.g2o'
    gfile = '/home/nuc/graph/fullGraph_renamed.g2o'
    gfile = '/home/nuc/graph/fullGraph_opt_renamed_opt_renamed.g2o'
    gfile = '/home/nuc/github/lusha/Multi-robot-SLAM/MR_SLAM/Mapping/src/global_manager/log/full_graph_renamed.g2o'
    gfile = "/home/nuc/github/lusha/Multi-robot-SLAM/MR_SLAM/Mapping/src/global_manager/log/FixLagSmoother/fullGraph_opt_before.g2o"
    # gfile = "/home/nuc/github/jiangpin/Mapping/src/global_manager/log/FixLagSmoother/fullGraph_opt_before.g2o"
    gfile = "/home/nuc/github/jiangpin/Mapping/src/global_manager/log/FixLagSmoother/fullGraph_opt_after.g2o"
    gfile = "/home/nuc/github/jiangpin/Mapping/src/global_manager/log/full_graph.g2o"
    gfile = '/home/nuc/graph/multi-lio_result/05-16/Fix_fullGraph_opt_before_renamed.g2o'
    gfile = '/home/nuc/graph/multi-lio_result/05-16/full_graph_renamed.g2o'
    gfile = "/home/nuc/github/jiangpin/Mapping/src/global_manager/log/FixLagSmoother/fullGraph_opt_after.g2o"

    gfile = "/home/nuc/github/jiangpin/Mapping/src/global_manager/log/FixLagSmoother/fullGraph_opt_after.g2o"

    # gfile = "/home/nuc/github/jiangpin/Mapping/src/global_manager/log/FixLagSmoother/fullGraph_opt_before.g2o"


    # gfile = "/home/nuc/github/jiangpin/Mapping/src/global_manager/log/full_graph.g2o"
    # gfile = "/home/nuc/github/lusha/Multi-robot-SLAM/MR_SLAM/Mapping/src/global_manager/log/FixLagSmoother/fullGraph_opt_after.g2o"

    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/FixLagSmoother/result_renamed.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/full_graph_renamed.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/full_graph_renamed.g2o"

    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/gnc/result.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/past/3_robot_3/full_graph.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/past/3_robot_full/full_graph.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/3_robot/full_graph.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot/full_graph.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/3_robot/result_renamed.g2o"

    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot/remain_optimized.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot/remain_optimized.g2o"

    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/zy_2robot_020/result.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/zy_2robot_020/full_graph.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/zy_2robot_020/skeleton.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/3_robot/result.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/zy_2robot_020/result.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/104_2robot/result.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/3_robot/result.g2o"

    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot/remain_optimized.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/EGNC-PGO/experiments/data/3_robot/full_graph.g2o"


    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/zy_2robot/remain_optimized.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/zy_2robot/full_graph_optimized.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/zy_2robot/full_graph.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/zy_2robot/skeleton.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/zy_2robot/skeleton_optimized.g2o"

    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/zy_2robot/full_graph.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/zy_2robot/result.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/test_opt.g2o"

    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/3_robot/full_graph.g2o"
    
    
    
    # gfile = "/home/jiangpin/dataset/Darpa/tunnel-20230811T100846Z-001/tunnel/g2o_pcd/result.g2o"
    # gfile = "/home/jiangpin/dataset/Darpa/ku-20230811T101027Z-001/ku/g2o_pcd/result.g2o"
    
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/104_2robot/full_graph.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/104_2robot/remain_optimized.g2o"

    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/EGNC-PGO/experiments/data/3_robot_005/remain_optimized.g2o"

    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot_005/full_graph.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot_005/remain.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot_005/remain_optimized.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot_005/remain_optimized.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/3_robot/result.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot_005/skeleton.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot/full_graph.g2o"
    
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/srrg2_ws/data/3_robot/opt_reorderd.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/S3E_clollege/full_graph.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/S3E_clollege_010/full_graph.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/S3E_clollege_025/full_graph.g2o"
    # gfile = "/home/jiangpin/dataset/s3e/full_graph.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/S3E_clollege_025/result.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Multi-Robot-LIO/src/global_manager/log/S3E_college_030/full_graph.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/S3E_clollege_010/result.g2o"

    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/S3E_clollege_025/result.g2o"
    # gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/S3E_clollege_025/full_.g2o"
    gfile = "/home/jiangpin/projects/Multi-Robot-SLAM/Kimera-RPGO/data/S3E_clollege_025/fullGraph_opt_after.g2o"
    gfile = "/home/jiangpin/dataset/s3e/fullGraph_opt_after.g2o"


    












    if is_file_renamed(gfile):
        pass
    else:
        g2o_tool = G2oTool()
        _,_ = g2o_tool.read(gfile)
        g2o_tool.rename_id()
        # g2o_tool.reorder_id()
        str1,str2 = gfile.split('.')
        gfile = str1+'_renamed.g2o'

    graph = PoseGraph3D(verbose=True,use_transform=False)
    graph.load_file(gfile)
    print("loaded:"+gfile)
    viewer = MultiViewer3D(graph,3,gfile)