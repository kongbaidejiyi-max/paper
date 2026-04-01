# Global reference path for UGV (UAV → UGV)
std_msgs/Header header
uint32 map_version                # 地图/路径版本（UAV 侧每次接收新图自增）
geometry_msgs/Pose[] poses        # 路径控制点（世界系）
float32[] corridor_half_width     # 对应poses的走廊半宽 w_i，单位m
float32[] path_confidence         # 对应poses的段置信度 c_i ∈ [0,1]
