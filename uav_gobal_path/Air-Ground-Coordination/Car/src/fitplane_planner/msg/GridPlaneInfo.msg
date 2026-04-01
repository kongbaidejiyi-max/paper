Header header
float32 resolution
uint32 width
uint32 height
geometry_msgs/Point origin
float32[] traversability  # 每个栅格的可通行性
float32[] plane_angle    # 每个栅格的平面角度
float32[] plane_height   # 每个栅格的高度
int32[] occupancy       # 栅格占用状态：-1未知，0空闲，100占用 