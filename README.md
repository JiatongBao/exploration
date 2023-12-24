# An Improved Dual-Stage Viewpoint Planner for Rapid Exploration by Frontier Clustering and Tour Planning

This repository will provide C++ code for autonomous exploration in both simulation and real-world settings on a mobile robot. This is the reference implementation for the paper which has been submitted on <i>T-ASE</i>.

### An Improved Dual-Stage Viewpoint Planner for Rapid Exploration by Frontier Clustering and Tour Planning

Autonomous exploration in unknown environments has become a critical capability of mobile robots. Most methods detect frontiers, sample viewpoints, and navigate the robot towards the viewpoint with maximum information gain. However, these methods often suffer from problems such as viewpoint selection based solely on information gain and inefficient tour optimization. To address these issues, an improved Dual-Stage Viewpoint Planner (DSVP) has been proposed, which includes frontier clustering and tour planning for efficient exploration in highly convoluted environments. Our method involves two stages: exploration and relocation. During the exploration stage, we select frontiers from a local Rapidly-exploring Random Tree (RRT) without any bias, cluster them into different connected spaces, and identify individual candidate goals for each cluster. After performing tour planning on these local candidate goals, the exploration boundary is rapidly expanded. To enhance the robustness of our method, a retrying mechanism is proposed to regenerate the RRT for local exploration before the robot switches to the relocation stage. All local candidate goals that have not been visited before are taken as global goals and clustered for global tour planning. The robot is then navigated to a previously recognized but unexplored area based on the planned global tour. Both stages maintain a local viewpoint graph and a global viewpoint graph, respectively, to find the shortest path between two given positions. The two stages are performed back and forth until no global candidate goals remain. The proposed method is compared to other related methods in various challenging simulated and real-world environments. Experimental results demonstrate that our method is more effective and efficient in space exploration than the compared methods. 

<!-- ![Method Overview](method.png?raw=true) -->
<div align="center"><img src="images/method.png" width="80%"/></div>
Fig.1 Overview of our method. The example environment which the robot is required to explore is shown as a 2D map. Explored areas are depicted in blue, while unexplored regions are represented by white spaces. Our method comprises two stages: the exploration stage and the relocation stage. During the exploration stage, an RRT is expanded within the local planning horizon, where each node serves as a viewpoint. Frontiers are identified from these viewpoints. The identified local frontiers are clustered, with each cluster corresponding to a local candidate goal. By using local tour planning, the optimal local candidate goal is selected as the subsequent navigation target, considering the remaining goals as global ones. Once no frontier clusters in the local planning horizon remain, the robot switches to the relocation stage. In this stage, all updated global candidate goals are employed for global tour planning. The robot is then guided towards the selected global goal. These two stages are executed back-and-forth until no global candidate goals remain.

## Videos
<b> Exploring in the simulated environments</b>

To be available.

<b>Exploring in a real underground parking lot environment</b>
<div align="center"><img src="images/keyframes.gif" width="100%"/></div>
Note: We took screenshots throughout our exploration and later compiled these key moments into a brief video.


## Contact
If you have any questions or find any bugs, please let me know: [Jiatong Bao] jtbao[at]yzu[dot]edu[dot]cn


## Acknowledgement
Our code is based on [DSVP](https://github.com/HongbiaoZ/dsv_planner).