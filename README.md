# Multi-Modal Teach & Repeat  

## Overview  

This package is part of the navigation stack developed for IUK Agri-OpenCore. It provides an enhanced teach-and-repeat module for robotic navigation using a multi-modal approach. RGB-D and laser data are recorded during the teaching phase and compared with real-time data to adjust the robot's alignment during the repeat phase.  

Classical feature matching techniques, such as ORB and SIFT, are applied to RGB images to ensure robust and generalized performance. Additionally, KNN is used for laser data comparison, improving the accuracy of alignment corrections. Further details will be included in upcoming documentation.  

## Installation  

To set up **teachrepeat** in your ROS 2 workspace, follow these steps:  

1. Clone the repository into your workspace:  
    ```bash  
    git clone https://github.com/LCAS/teachrepeat.git
    cd teachrepeat  
    ```  

