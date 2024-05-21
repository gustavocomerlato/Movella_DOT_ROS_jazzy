# Movella_DOT_ROS

### SDK Setup

- This is the installation guide to how you can setup Movella DOT sensors in your Linux PC.
- Couple of attachments to refer in the process of setup are:
    1. Xsens Dot User Manual - [https://www.movella.com/hubfs/Xsens DOT User Manual.pdf?hsCtaTracking=fdf42b03-0506-4282-ab58-4995b3251a37|b467fb0f-af45-4d46-b027-38df0a26d8ec](https://www.movella.com/hubfs/Xsens%20DOT%20User%20Manual.pdf?hsCtaTracking=fdf42b03-0506-4282-ab58-4995b3251a37%7Cb467fb0f-af45-4d46-b027-38df0a26d8ec)
    2. Movella DOT SDK guide - https://base.movella.com/s/article/Movella-DOT-PC-SDK-Guide?language=en_US
    3. Movella DOT Quick Start guide - https://base.movella.com/s/article/Movella-DOT-Quick-Start-Guide?language=en_US
    4. Website to download the SDK  - Please download the latest Movella DOT SDK for Linux

[Software Downloads | Movella.com](https://www.movella.com/support/software-documentation?hsCtaTracking=39d661fa-2ea8-4478-955e-01d0d8885f14|3ad1c7d6-9c3a-42e9-b424-5b15b9d0924e)

- Once you download the SDK, extract the folder and go through the folder until you find ‘movelladotpcsdk_linux-x64_2023.6.sh’. Open a new terminal in that folder location and type:
    1. If you want to install it in the default location that it shows on the screen - ‘/usr/local/movella’ 
    
    ```jsx
    sudo ./movelladotpcsdk_linux-x64_2023.6.sh
    ```
    
    **OR**
    
    1. If you want to install it in a different location other than the root partition of your computer. 
    
    ```jsx
    ./movelladotpcsdk_linux-x64_2023.6.sh
    ```
    

You may experience an error like

```cpp
'uudecode' could not be found. It is usually installed with the 'sharutils' package
```

If so, run the below command:

```cpp
sudo apt-get install sharutils
```

- In my setup, I have used the default location (this is also reflected in the ros-workspace. If you are installing in a different location, then you will need to make the changes accordingly in the ros-workspace)
- Once the movella sdk is installed, you can try out the example - “movelladot_pc_sdk_receive_data.py”. First, navigate to the folder where the python example scripts sits. This is inside the examples folder inside the movella folder in the default installation location - “/usr/local/”. After navigating to the required folder, open the README file and follow the instructions shown in it. It says:
    - Install python 3.9 (recommended), but not less than 3.7. If you have another version between 3.7 and 3.10, it should be fine.
    - Execute ‘pip install -r requirements.txt’ in a terminal in that folder location.
    - After running the above command navigate to /usr/local/movella/python and do the next step.
    - Run “pip install movelladot_pc_sdk-2021.1.0-cp**39**-none-win_amd64.whl” in the same terminal. This depends on the python version that you have in your PC. Change the bold numbers in the command shown according to the python version installed in your PC. If it is python 3.8, then the bold numbers change to **38.** It can vary from **37** to **310.**
    
    After running the above steps, navigate to /usr/local/movella/examples/xdpcsdk/python and run the example in the same terminal with the command:
    

```jsx
python3 movelladot_pc_sdk_receive_data.py
```

- Make sure at least one of the DOT sensors are turned on. Refer to the quick start guide or user manual on how to turn on and off the sensor. You should see it connect to the sensor and the roll, pitch and yaw values should be printed on the screen. This means the SDK is working fine.

### ROS-Workspace

The ROS workspace is created by following the ros workspace tutorials. You can download the workspace that I created from this [github repo](https://github.com/baslinjames/Movella_DOT_ROS/tree/master). Clone the repo to your computer (Home directory) using this command:

```cpp
git clone https://github.com/baslinjames/Movella_DOT_ROS.git
```

I am assuming that you have ros-noetic and catkin tools already installed in your PC. You can install ros-noetic from [here](http://wiki.ros.org/noetic/Installation/Ubuntu). Follow the recommended instructions. To install catkin tools, go to this [webpage](https://catkin-tools.readthedocs.io/en/latest/installing.html) and follow the instructions with ‘apt-get’. Open the cloned workspace in VS-Code or any other editor of your choice. Navigate to CMakelists.txt and comment these lines:

```cpp
add_executable(ros_movella src/ros_movella.cpp)
target_link_libraries(ros_movella ${catkin_LIBRARIES}
    /usr/local/movella/lib/libmovelladot_pc_sdk.so
    /usr/local/movella/lib/libxstypes.so.2022
    /usr/local/movella/examples/xdpcsdk/cpp/xdpchandler.cpp.o
    /lib/x86_64-linux-gnu/libpthread.so.0
    /usr/local/movella/examples/xdpcsdk/cpp/conio.c
)
```

Then in the same terminal, navigate to Movella_DOT_ROS workspace:

```cpp
cd Movella_DOT_ROS
```

and run:

```cpp
catkin build
```

This should be successful. Now, you need to change one line in the ros_movella.cpp file:

```cpp
#include "/home/project/Movella_DOT_ROS/devel/include/movella_dot/DotSensorMsg.h"
```

You will need to change **project** in the above command to your PC’s username. You can now uncomment the lines that you had commented out and save both the changes. Since we are using the cpp library, we need to execute the make file in the movella sdk directory. Navigate to /usr/local/movella/examples/xdpcsdk and open a terminal and run:

```cpp
sudo make
```

Now go back to the other terminal where our ros workspace is opened and run:

```cpp
catkin build
```

Once you run ‘catkin build’, it should build successfully if you have installed the movella SDK in the default location as described in the previous section. If you have installed the movella SDK in a different location, please make the changes in the CMakelists.txt file in the workspace, because when we create the cpp/python file, we link multiple libraries to the this code and we specify the path to the movella SDK.

Before running the ROS code, we need to pair the dot sensors with the PC. Follow these steps:

1. Open the Bluetooth settings on your PC.
2. Turn on each dot sensor one at a time and pair them.
3. All dot sensors will be named 'Xsens DOT'. After pairing a sensor, turn Bluetooth off and then back on.
4. If a sensor name shows as 'Disconnected', it means the sensor is paired but not currently connected.
5. If it shows as 'Not setup', that is the sensor that needs to be paired.

Once you pair all the required dot sensors, turn the bluetooth off and on again. Do not connect to the dot sensors manually through the settings tab. The dot sensors will be connected when you run the code. Have a look at the ‘ros_movella.cpp’ file, it is the same as the example in the movella SDK named ‘example_receive_data.cpp’. The only changes that I have brought in is the ros integration and checking each sensors bluetooth address, as they remain static. Run the below command in the workspace directory in terminal:

```cpp
rosrun movella_dot ros_movella
```

This should first show the number of devices that you are trying to connect and then establishes the connection with them. It then shows which dot sensor is connected on the terminal. I have randomly numbered them from 1-5 for the five addresses. Once that is done, you can open a new terminal and run these commands:

```cpp
source devel/setup.bash
rostopic echo /DOT_Pos
```

This should now display the euler rotations of each of the dot sensors. If you want to publish other type of values, refer the SDK’s API for functions relating to your needs and apply them in the code and publish it as a ros topic.
