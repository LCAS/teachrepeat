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

3. In the GUI, enter the map name and use the following controls:  
    - **Map Name**: Enter the desired map name during mapping, or select an existing map from the dropdown list.  
    - **Record Interval**: Use the slider to set the data recording/comparison interval in seconds.  
    - **Start Mapping**: Begin the mapping process and drive the robot to desired locations.  
    - **Stop Mapping**: Save the map and stop the mapping process.  
    - **Repeat Mapping**: Return the robot to the starting position and replay the map.  
    - **Plot Map**: Visualize odometry data by clicking this button after repeating the map. The plot will display in a separate window.  

### Script-Based Control  

A script (`map_control.sh`) is available for mapping and repeating actions via the command line.  

- **Start Mapping**:  
    ```bash  
    ./map_control.sh start_map <name> <interval>  
    ```  
    Replace `<name>` with the map name and `<interval>` with the desired recording interval (in seconds). Drive the robot after initiating this command.  

- **Stop Mapping**:  
    ```bash  
    ./map_control.sh stop_map <name> <interval>  
    ```  
    This will save the map and stop the mapping process.  

- **Repeat a Saved Map**:  
    ```bash  
    ./map_control.sh repeat_map <name> <interval>  
    ```  
    This will replay the previously recorded map.  

### Important Notes  

- Ensure that the relevant ROS 2 topics (e.g., `/cmd_vel`, `/odom`, `/camera/image_raw`) are correctly mapped in your launch files based on your robot’s configuration.  
- Mapping and repeating are implemented as ROS 2 actions, allowing easy integration with other components.
- Update `<ws>` in the `map_gui.py` to your own workspace in order to use the script.
- This Package relies on velocity command like `cmd_vel` to record the velocities.
- The **Plot Map** feature relies on odometry data accuracy. Even if the robot repeats the map correctly, the plot may appear inconsistent if odometry drift occurs.  
