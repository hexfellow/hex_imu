# **hex_imu**

## **Overview**

This **hex_imu** repository provides an implementation of IMU data acquisition and processing based on CANopen protocol.

### **License**

This project is licensed under the terms of the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

### **Maintainer**

- **[Zhang Jianhuai](https://github.com/aalicecc)**

### **Supported Platform**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**

### **Supported ROS Version**

- [x] **ROS Noetic**
- [x] **ROS Humble**

---

## **Public APIs**

### **Publish**

| Topic              | Msg Type                  | Description                                |
| ------------------ | ------------------------- | ------------------------------------------ |
| `/imu_data`        | `sensor_msgs/Imu`         | IMU data including orientation, angular velocity and linear acceleration |
| `/magnetic_data`   | `sensor_msgs/MagneticField` | Magnetic field data |

### **Subscribe**

| Topic    | Msg Type                      | Description                           |
| -------- | ----------------------------- | ------------------------------------- |
| None     | None                          | No subscription required              |

### **Parameters**

| Name                    | Data Type             | Description                                                                                |
| ----------------------- | --------------------- | ------------------------------------------------------------------------------------------ |
| `node_id`              | `int`                 | CANopen node ID of the IMU device                                                          |
| `channel`              | `string`              | CAN channel name (e.g., 'can0')                                                            |
| `imu_topic`            | `string`              | Topic name for publishing IMU data                                                         |
| `magnetic_topic`       | `string`              | Topic name for publishing magnetic field data                                              |

---

## **Getting Started**

### **Dependencies**

- This software is built on the **ROS Noetic** or **ROS Humble**, which needs to be installed first.  
- **Package**
	- **python-can**
	- [**hex_utils**](https://github.com/hexfellow/hex_utils)
- **plugin**
	- **[rviz_imu_plugin](https://index.ros.org/p/rviz_imu_plugin)**
### **Install**

1. Create a workspace `catkin_ws` and get into the `src`.

	```shell
	mkdir -p catkin_ws/src
	cd catkin_ws/src
	```

2. Clone this repo.

	```shell
	git clone git@github.com:hexfellow/hex_imu.git
   ```

3. Go to `catkin_ws` directory and build the repo.

	- **ROS1**
	
		```shell
		cd ../
		catkin_make
		```
	
	- **ROS2**
	
		```shell
		cd ../
		colcon build
		```

4. Source the `setup.bash` and run the test below

	- **ROS 1**
	    
	    ```shell
		source devel/setup.bash --extend
	    ```
	    
	- **ROS 2**
	    
	    ```shell
		source install/setup.bash --extend
	    ```
### **Usage**

1. Launch the main node:

	- **ROS 1**
        
        ```shell
		roslaunch hex_imu canopen_imu.launch
        ```
        
    - **ROS 2**
        
        ```shell
		ros2 launch hex_imu canopen_imu.launch.py
        ```

2. We also offer a dedicated launch file for visualizing IMU data. Follow the steps below to get started:
	
	Before you begin, make sure you have the necessary plugin installed for RViz:
	- **ROS 1**
		 
	```
		sudo apt install ros-noetic-rviz-imu-plugin
	```
		
	- **ROS 2**
		
	```
		sudo apt install ros-humble-rviz-imu-plugin
	```
	Once the plugin is installed, you can start the visualization tool with the following command:
	- **ROS 1**
        
        ```shell
		roslaunch hex_imu canopen_imu_display.launch
        ```
        
    - **ROS 2**
        
        ```shell
		ros2 launch hex_imu canopen_imu_display.launch.py
        ```

3. If you want to output Azimuth, you can run the following sample code (only supports ROS1)

        
	```shell
	roslaunch hex_imu canopen_imu.launch

	rosrun hex_imu example.py
	```