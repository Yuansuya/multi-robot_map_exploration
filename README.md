# multi-robot_map_exploration-
## Introdution
**該source code為論文Blockchain-Based Multi-Robot Collaborative Map Exploration @NCU by Kevin Zhang (2023)**

使用ros2作為機器人控制系統，需要完成自主探索的方法，其中包含尋找任務點，導航還有跟區塊鏈做交互([blockchain source code](https://github.com/Yuansuya/fabric_ros2_multi-robot_map_exploration))。

採用gazebo作為模擬軟體。[Demo影片](https://www.youtube.com/watch?v=X8ZK3-JHJ0A&t=13s)

Version:
* Ubuntu: 20.04
* ros2: foxy
* hyperledger fabric: 2.5


## packages

### frontier_detector_pkg
- **frontier_detector_pkg/frontier_detector_pkg/detector.py**
- **frontier_detector_pkg/frontier_detector_pkg/functions.py**

主要用來找尋新的任務點的pakcage，使用方法是frontier-based。

### navi
#### **navi/navi/action_navi_node.py**
導航的主程式(兩點間移動)，並且移動的過程會產生新的走過的路徑點，可以設定time_threshold參數來控制在走動的過程中每隔多久生一個路徑點
#### **navi/naiv/testGoalPose.py**
整個系統的主程式(啟動點)，要跟以下不同的package交互
* 找尋任務點(新的未探索點)
* navigation
* blockchain
  - update topological map
  - update task points

#### vis_point
- vis_point/vis_point/vis_tm.py
- vis_point/vis_point/vis_tra.py
- vis_point/vis_point/visualize.py
主要用於資料視覺化，讓我可以在Rviz上觀察
#### my_interfaces
用於擴充ros2的可傳送service的資料結構，擴充的有
- my_interfaces/srv/FindFrontier.srv
- my_interfaces/srv/MoveGoal.srv
