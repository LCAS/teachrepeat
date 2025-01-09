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

2. Install required Python dependencies:  
    ```bash  
    pip install -r requirements.txt  
    ```  

3. Navigate to your workspace and build the package:  
    ```bash  
    colcon build --packages-select teachrepeat  
    ```  
## Usage  

### GUI Control  

**teachrepeat** includes a simple GUI for controlling mapping and repeating actions. To launch the GUI:  

1. Source the setup.bash file and launch the ROS 2 nodes:  
    ```bash  
    ros2 launch teachrepeat launch_teachrepeat.py  
    ```  

2. Run the GUI script from the package directory:  
    ```bash  
    python3 map_gui.py  
    ```  

    This will open the GUI interface.  

    ![Selection_002](https://github.com/user-attachments/assets/2f47dcd8-b0b3-4c70-85b1-8b88a1e42f31)  