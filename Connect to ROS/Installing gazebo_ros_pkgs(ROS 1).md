## 安装 `gagazebo_ros_pkgs`(ROS 1)
## 介绍

用于与 Gazebo 接口的一套 ROS 包包含在一个名为`gazebo_ros_pkgs`的新元包中。在继续这里之前，请看 [ROS overview](./ROS%20overview.md) 以了解背景信息。

这些说明是针对使用与ROS Lunar、ROS Kinetic和ROS Indigo完全集成的Gazebo版本。建议在继续阅读本教程之前，先阅读ROS/Gazebo版本的哪个组合。根据你的需求，你可能需要另一种安装方式。

## 先决条件

你应该了解ROS的基本概念，并通过ROS教程。

### 安装 ROS

我们推荐您安装这些ROS集成教程（`ros-lunar-desktop-full`、`ros-kinetic-desktop-full`或`ros-indigo-desktop-full`），以便您拥有所有必要的包。

请参阅[ROS安装页面](http://www.ros.org/wiki/ROS/Installation)了解更多细节。请确保按照ROS安装页面上的说明来编写ROS setup.bash脚本。

### 安装 Gazebo

你可以从源码或预构建的Ubuntu debians中安装Gazebo

参见[Install Gazebo](http://gazebosim.org/tutorials?cat=install)。如果从源码安装，请确保建立 gazebo_X.Y (X.Y是你想要的版本)分支

#### 测试 Gazebo 是否安装成功

在尝试安装 `gazebo_ros_pkgs` 之前，通过在终端中运行确保 Gazebo 工作。

```sh
gazebo
```

你应该会看到GUI打开的是一个空的世界。另外，通过点击左侧的“插入”选项卡并选择要添加的模型来测试添加模型（然后点击模拟来选择放置模型的位置）

#### 确保你安装了正确的 Gazebo 版本

要查看你安装Gazebo的位置，以及它是否在正确的位置，运行：

```sh
which gzserver
which gzclient
```

如果你是从源码安装的，那么默认位置为：

```sh
/usr/local/bin/gzserver
/usr/local/bin/gzclient
```

如果你是从包安装的：

```sh
/usr/bin/gzserver
/usr/bin/gzclient
```

### 安装 `gazebo_ros_pkgs`

选择你喜欢的方法。最简单快捷的方法是从包中安装，但从源码中安装意味着你可以更容易地调试和提交错误补丁;-)

#### A. 安装

`gazebo_ros_pkgs` 可在以下安装

+ [ROS Lunar](http://ros.org/wiki/lunar):

```sh
sudo apt-get install ros-lunar-gazebo-ros-pkgs ros-lunar-gazebo-ros-control
```

+ [ROS Kinetic](http://ros.org/wiki/kinetic):

```sh
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```

+ [ROS Indigo](http://ros.org/wiki/indigo):

```sh
sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
```

如果你这个安装方法成功结束，请跳转到下面的测试 Gazebo 与 ROS 集成部分

#### B. 从源码安装（Ubuntu）

如果你运行的是ROS的早期版本，你需要从源头安装`gazebo_ros_pkgs`。如果你想开发新的插件或提交补丁，从源头安装也很有用

##### 建立一个 Catkin 工作空间

你可以通过下面的命令，创建一个工作空间：

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```

然后在你的.bashrc文件中添加一个安装脚本的源：

```sh
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

更多细节参见[Create A Catkin Workspace](http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace)

##### 克隆 Github 存储库

确保你的设备上安装了`git`

```sh
sudo apt-get install git
```

**ROS Lunar**

Lunar 使用的是 gazebo7 系列，从安装它开始：

```sh
sudo apt-get install -y libgazebo7-dev
```

从 `gazebo_ros_pkgs` [github reponsitory](https://github.com/ros-simulation/gazebo_ros_pkgs)下载源代码

```sh
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel
```

使用rosdep检查任何缺失的依赖关系

```sh
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro kinetic
```

你可以通过debian install使用rosdep自动安装缺少的依赖关系

```sh
rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
```

现在转到[build the gazebo_ros_pkgs](http://gazebosim.org/#Buildthegazebo_ros_pkgs)章节

**ROS Indigo**

Indigo 使用的是 gazebo2 系列，从安装它开始：

```sh
sudo apt-get install -y gazebo2
```

从 `gazebo_ros_pkgs` [github reponsitory](https://github.com/ros-simulation/gazebo_ros_pkgs)下载源代码

```sh
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b indigo-devel
```

使用rosdep检查任何缺失的依赖关系

```sh
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro indigo
```

你可以通过debian install使用rosdep自动安装缺少的依赖关系

```sh
rosdep install --from-paths . --ignore-src --rosdistro indigo -y
```

现在转到[build the gazebo_ros_pkgs](http://gazebosim.org/#Buildthegazebo_ros_pkgs)章节

##### 构建`gazebo_ros_pkgs`

要构建Gazebo ROS集成包，请运行以下命令：

```sh
cd ~/catkin_ws/
catkin_make
```

#### 使用 ROS 测试 Gazebo

请确保始终以适当的 ROS 设置文件为源，Kinetic 的设置文件是这样做的

```sh
source /opt/ros/kinetic/setup.bash
```

你可能想把这一行添加到你的`~/.bashrc`中。

假设你的 ROS 和 Gazebo 环境已经被正确设置和构建，你现在应该可以在启动`roscore`之后，通过一个简单的`rosrun`命令运行`Gazebo`。

如果你的`.bashrc`里没有 atkin setup.bash 的话，直接 source setup.bash

```sh
source ~/catkin_ws/devel/setup.bash
```
```sh
roscore &
rosrun gazebo_ros gazebo
```

Gazebo Gui 应该出现并且里面没有任何东西

![]()

要验证是否设置了正确的ROS连接，请查看可用的 ROS topic：

```sh
rostopic list
```

你应该可以看到如下话题：

```sh
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
```

你也可以验证 Gazebo services 的存在：

```sh
rosservices list
```

你应该可以看到如下服务：

```sh
/gazebo/apply_body_wrench
/gazebo/apply_joint_effort
/gazebo/clear_body_wrenches
/gazebo/clear_joint_forces
/gazebo/delete_model
/gazebo/get_joint_properties
/gazebo/get_link_properties
/gazebo/get_link_state
/gazebo/get_loggers
/gazebo/get_model_properties
/gazebo/get_model_state
/gazebo/get_physics_properties
/gazebo/get_world_properties
/gazebo/pause_physics
/gazebo/reset_simulation
/gazebo/reset_world
/gazebo/set_joint_properties
/gazebo/set_link_properties
/gazebo/set_link_state
/gazebo/set_logger_level
/gazebo/set_model_configuration
/gazebo/set_model_state
/gazebo/set_parameters
/gazebo/set_physics_properties
/gazebo/spawn_gazebo_model
/gazebo/spawn_sdf_model
/gazebo/spawn_urdf_model
/gazebo/unpause_physics
/rosout/get_loggers
/rosout/set_logger_level
```

#### 其它 ROS 方法去启动 Gazebo

有几个启动 Gazebo 的 rosrun 命令：

+ 同时启动服务和客户端：

```sh
rosrun gazebo_ros gazebo
```

+ 只启动服务端

```sh
rosrun gazebo_ros gzserver
```

+ 只启动客户端

```sh
rosrun gazebo_ros gzclient
```

+ 只启动 Gazebo 服务，在调试模式下使用GDB

```sh
rosrun gazebo_run debug
```

+ 此外，你可以使用`roslaunch`启动 Gazebo

```sh
roslaunch gazebo_ros empty_world.launch
```

