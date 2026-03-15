# perception_interfaces/msg/ObjectInfo.msg

int32 class_id           # 클래스 ID (0:Unknown, 1:Car, 9:Pedestrian 등)
float32 score            # 인식 신뢰도 (0.0 ~ 1.0)

float32 velocity_x       # x축 속도 (m/s)
float32 velocity_y       # y축 속도 (m/s)

geometry_msgs/Pose pose          # 중심 좌표(x,y,z) 및 회전(quaternion)
geometry_msgs/Vector3 dimensions # 물체 크기 (x, y, z)