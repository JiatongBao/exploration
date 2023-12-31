# An Improved Dual-Stage Viewpoint Planner for Rapid Exploration by Frontier Clustering and Tour Planning

This repository will provide C++ code for autonomous exploration in both simulation and real-world settings on a mobile robot. This is the reference implementation for the paper which has been submitted on <i>Robotica</i>.

### An Improved Dual-Stage Viewpoint Planner for Rapid Exploration by Frontier Clustering and Tour Planning

Autonomous exploration in unknown environments has become a critical capability of mobile robots. Most methods detect frontiers, sample viewpoints, and navigate the robot towards the viewpoint with maximum information gain. However, these methods often suffer from problems such as viewpoint selection based solely on information gain and inefficient tour optimization. To address these issues, an improved Dual-Stage Viewpoint Planner (DSVP) has been proposed, which includes frontier clustering and tour planning for efficient exploration in highly convoluted environments. Our method involves two stages: exploration and relocation. During the exploration stage, we select frontiers from a local Rapidly-exploring Random Tree (RRT) without any bias, cluster them into different connected spaces, and identify individual candidate goals for each cluster. After performing tour planning on these local candidate goals, the exploration boundary is rapidly expanded. To enhance the robustness of our method, a retrying mechanism is proposed to regenerate the RRT for local exploration before the robot switches to the relocation stage. All local candidate goals that have not been visited before are taken as global goals and clustered for global tour planning. The robot is then navigated to a previously recognized but unexplored area based on the planned global tour. Both stages maintain a local viewpoint graph and a global viewpoint graph, respectively, to find the shortest path between two given positions. The two stages are performed back and forth until no global candidate goals remain. The proposed method is compared to other related methods in various challenging simulated and real-world environments. Experimental results demonstrate that our method is more effective and efficient in space exploration than the compared methods. 

<!-- ![Method Overview](method.png?raw=true) -->
<div align="center"><img src="images/method.png" width="80%"/></div>
Fig.1 Overview of our method. The example environment which the robot is required to explore is shown as a 2D map. Explored areas are depicted in blue, while unexplored regions are represented by white spaces. Our method comprises two stages: the exploration stage and the relocation stage. During the exploration stage, an RRT is expanded within the local planning horizon, where each node serves as a viewpoint. Frontiers are identified from these viewpoints. The identified local frontiers are clustered, with each cluster corresponding to a local candidate goal. By using local tour planning, the optimal local candidate goal is selected as the subsequent navigation target, considering the remaining goals as global ones. Once no frontier clusters in the local planning horizon remain, the robot switches to the relocation stage. In this stage, all updated global candidate goals are employed for global tour planning. The robot is then guided towards the selected global goal. These two stages are executed back-and-forth until no global candidate goals remain.

## Videos
<b> Exploring in the simulated environments</b>

[![ Explore in the simulated indoor environment ](https://res.cloudinary.com/marcomontalbano/image/upload/v1703407943/video_to_markdown/images/youtube--6gFU1xZ8MiU-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://www.youtube.com/watch?v=6gFU1xZ8MiU " Explore in the simulated indoor environment ")

<b>Exploring in a real underground parking lot environment</b>
<div align="center"><img src="images/keyframes.gif" width="100%"/></div>
Note: We took screenshots throughout our exploration and later compiled these key moments into a brief video.

## How to compile

- edit the "def.h"
- enable "#define NEW_METHOD" for running our method
- disable "#define NEW_METHOD" for running the original DSVP method

```
cd dsv_planner
catkin_make
```

## How to run
- launch the simulated environment
```
roslaunch vehicle_simulator system_indoor.launch
```
- run the exploration planner
```
cd dsv_planner
catkin_make
source devel/setup.bash
roslaunch dsvp_launch explore_indoor.launch
```
## Contact
If you have any questions or find any bugs, please let me know: [Jiatong Bao] jtbao[at]yzu[dot]edu[dot]cn


## Acknowledgement
The benchmark exploration dataset is from [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment).

Our planner code is based on [DSVP](https://github.com/HongbiaoZ/dsv_planner).
