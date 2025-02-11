# The movella_dot_ros package, Jazzy version

The movella_dot_ros metapackage is a ROS 2 metapackage to interface with [Movella's XSens DOT IMUs](https://www.movella.com/products/wearables/movella-dot). This metapackage is a port to ROS 2 Jazzy from the [baslinjames' original Movella_DOT_ROS metapackage for ROS Noetic](https://github.com/baslinjames/Movella_DOT_ROS), with a few changes and other enhancements. I am deeply indebted to his contributions and tutorial on how to get the Movella DOTS up and running with my PC.

The metapackage is split into the packages movella_dot and movella_msgs packages. The package movella_dot contain the implementation of a ROS 2 "driver" to publish IMU readings in three topics (/IMU, /magnetic_field and the custom /dot topic). The movella_msgs package describes the DotSensor message format, used in the /dot topic, which contains all available sensor readings in one single packet.

### SDK Setup

This package uses the [Movella DOT SDK](https://base.movella.com/s/article/Movella-DOT-PC-SDK-Guide?language=en_US) and the custom xdpcHandler implemented therein to manage the DOTs. Therefore, please install the Movella DOT SDK for Linux from [here](https://www.movella.com/support/software-documentation) if you haven't already done so. Extract it and install to a place of your choosing (on Ubuntu Linux 24.04 the default is /usr/local/movella).

Once installed, you can try out the examples contained therein, in either python, Java or C++. To build the C++ shared libs and executables, go to the /examples/xdpcsdk folder and run `sudo make`:

```bash
cd /usr/local/movella/examples/xdpcsdk # Change /usr/local/movella/ to your installation
sudo make
```

This should result in the files `conio.c` and `xdpchandler.cpp.o`. If the build fails in this step, you might be missing some libraries, such as `libcholmod.so.3`, `libumfpack.so.5` and `uudecode`, needed by the libxee_solver.so file. These can be installed by the following command:

```bash
sudo add-apt-repository -y -s deb http://security.ubuntu.com/ubuntu jammy main universe
# Use the command above if working on Ubuntu 24.04 or newer, to add the repos
sudo apt-get install libcholmod3 libumfpack5 sharutils
```

### ROS-Workspace

I am assuming that you have ROS 2 Jazzy and Colcon tools already installed in your PC. You can get ROS 2 Jazzy [here](https://docs.ros.org/en/jazzy/Installation.html), either from a binary installation or building from source. Follow the recommended instructions. To install colcon via apt, run the following command:

```bash
sudo apt install python3-colcon-common-extensions
```

Go to your ROS 2 workspace src directory, and create a directory named movella_dot_ros. There, clone [this repo](https://github.com/gustavocomerlato/Movella_DOT_ROS_jazzy/tree/master). This can be done with the following commands from `src/`:

```bash
mkdir movella_dot_ros && cd movella_dot_ros/
git clone https://github.com/gustavocomerlato/Movella_DOT_ROS_jazzy.git
```

If you have installed the Movella SDK anywhere else other than /usr/local/movella, open the movella_dot/CMakeLists.txt file in `vim`, `joe` or any other text editor of your choice, and find and replace `/usr/local/movella` below with your installation directory. 

```cpp
set(MOVELLA_DIR "/usr/local/movella")
```

The package is now ready to be built. Navigate to your ROS 2 workspace. Build and source the setup (don't forget to setup your ROS 2 environment first, if you haven't already done so). Then, build and source with the following commands:

```bash
colcon build --symlink-install
source ./install/setup.bash
```

And that's it for the installation of the package!

### Usage

To check if the package is working correctly, pen up a terminal, source the ROS 2 workspace, and run the `movella_dot` node with the following command:

```bash
ros2 run movella_dot movella_dot
```

Wait for all DOTS to be connected. Then, open up another terminal, and echo the output of any one DOT:

```bash
ros2 topic echo /dot_1/dot
```

You should be seeing quaternion, angular velocity, linear acceleration and magnetic field outputs at a rate of 60 Hz. If you want to publish other values or in other rates, refer to the SDK's API for functions related to your needs, and apply them in your code.