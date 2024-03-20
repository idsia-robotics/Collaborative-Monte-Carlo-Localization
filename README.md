# Resource-Aware Collaborative Monte Carlo Localization with Distribution Compression

## Abstract
 Global localization is essential in enabling robot autonomy, and collaborative localization is key for multi-robot systems. 
 In this paper, we address the task of collaborative global localization under computational and communication constraints. We propose a method which reduces the amount of information exchanged and the computational cost. We also analyze, implement and open-source seminal approaches, which we believe to be a valuable contribution to the community.  
  We exploit techniques for distribution compression in near-linear time, with error guarantees. 
  We evaluate our approach and the implemented baselines on multiple challenging scenarios, simulated and real-world. Our approach can run online on an onboard computer. We release an open-source C++/ROS2 implementation of our approach, as well as the baselines.
  <p align="center">
<img src="resources/motivation.png" width="800"/>
</p>

## Results
We conducted a thorough evaluation of the different approaches to cooperative localization. We present our experiments to show the capabilities of our method, Compress++ MCL. 
The results support the claims that our proposed approach (i) improves collaborative localization, (ii) decreases the required band-width, (iii) reduces the computational load, (iv) runs online
on an onboard computer.

  <p align="center">
<img src="resources/Successrateenv.png" width="800"/>
</p>

## Installation
We provide Docker installations for ROS 2 Humble.

## Running the Algo
Code is coming soon!
