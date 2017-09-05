# Automatic-Withdrawal
Code for reproducing key results in the paper [Automatic Withdrawal Method Based on 2D Laser Radar for Small Ground Mobile Robot](http://robot.sia.cn/CN/Y2017/V39/I5/688) by LI Jie, YUAN Xia, ZHAO Chunxia, LU Jianfeng.
# Abstract
For the withdrawaling problem of small ground mobile robots in the field, an automatic withdrawal system for the small ground mobile robot is designed and implemented. The 2D Laser radar is used for environmental perception and terminal precise navigation. Firstly, an adaptive curvature filter algorithm is proposed to process the radar data filtering. Then, in the process of guiding target detection, the scattered data are clustered according to the density and nearest adjacent distance measuring. The target detection and matching are carried out by using the geometric structure constraint between the guidance target and the auxiliary target. Finally, the real-time obstacle avoidance and the optimal forward direction selection are conduct based on the method of influence layer division and candidate direction estimation. Thus the robot is accurately guided to accomplish the automatic withdrawal. The developed automatic withdrawal system is validated in the natural environment. And some aspects of automatic withdrawal system are analyzed experimentally. Experiments show that the proposed method can effectively accomplish the autonomous withdrawal of the small ground mobile robot in the natural environment.
# Citation
[BibTeX](http://robot.sia.cn/CN/article/getTxtFile.do?fileType=BibTeX&id=15902) | [EndNote](http://robot.sia.cn/CN/article/getTxtFile.do?fileType=EndNote&id=15902) (RIS)    
李捷, 袁夏, 赵春霞, 陆建峰. 基于2维激光雷达的小型地面移动机器人自主回收方法[J]. 机器人, 2017, 39(5): 688-696.	
LI Jie, YUAN Xia, ZHAO Chunxia, LU Jianfeng. Automatic Withdrawal Method Based on 2D Laser Radar for Small Ground Mobile Robot. ROBOT, 2017, 39(5): 688-696.

# Code
核心算法包括：数据滤波算法；数据聚类算法；特定目标检测算法；影响层划分算法。 

说明：此处发布各算法的理论原型代码，采用matlab或python实现。在集成的实验平台上，采用C/C++实现，整体的系统运行与运动控制算法不在本文的范围之内。     

代码使用：
* 运行main.m的Matlab代码，即可实现将雷达数据进行滤波、聚类和特定目标检测。    
  
* 影响层划分算法对应layer.py的Python代码，运行该代码后将在控制台输出划分结果。
