# ROS noetic Tutorials (Ubuntu 20.04 desktop)



## 创建虚拟机后首先

调成不息屏 以防断网

software updater 更新

software & updates 设置换仓库



## 安装ROS

设置电脑以安装来自packages.ros.org的软件

用清华源

```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```

设置密钥

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

确保你的Debian软件包索引是最新的

```bash
sudo apt update
```

安装**完整桌面版安装（Desktop-Full，推荐）**：除了**桌面版**的全部组件外，还包括2D/3D模拟器（simulator）和2D/3D 感知包（perception package）。

```bash
sudo apt install ros-noetic-desktop-full
```

```bash
$ source /opt/ros/noetic/setup.bash
```

下面这些命令可以在每次启动新的shell窗口时很方便地为你自动source一下这个脚本（Bash）

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## 安装之后

首先检查是否正确配置环境变量

确保[ROS_ROOT](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_ROOT)和[ROS_PACKAGE_PATH](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_PACKAGE_PATH)这两个[环境变量](http://wiki.ros.org/ROS/EnvironmentVariables)正确设置

```bash
$ printenv | grep ROS
```

### 创建ROS(catkin)工作空间

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

接下来首先`source`一下新生成的`setup.*sh`文件

```bash
$ source devel/setup.bash
```

要保证工作区被安装脚本正确覆盖，需确定`ROS_PACKAGE_PATH`环境变量包含你当前的工作空间目录

```bash
$ echo $ROS_PACKAGE_PATH
/home/<username>/catkin_ws/src:/opt/ros/<distro>/share
```

### 一些常用的工具

```bash
rospack find [package_name] 
```

```bash
$ roscd [locationname[/subdir]] #到某个软件包或者软件包集当中
```

```bash
$ roscd log #存储ROS日志文件的目录
```

```bash
$ rosls [locationname[/subdir]]
```



## catkin 工作空间

```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

### 创建catkin软件包

先去刚刚那个空白的工作空间

```bash
cd ~/catkin_ws/src
```

创建一个软件包

```bash
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

```bash
catkin_create_pkg <package_name> [depend1] [depend2] [depend3] #catkin_create_package格式
```

make一下新建的软件包

```bash
$ cd ~/catkin_ws
$ catkin_make
```

工作空间构建完成后，在`devel`子目录下创建了一个与你通常在`/opt/ros/$ROSDISTRO_NAME`下看到的目录结构类似的结构。

要将这个工作空间添加到ROS环境中，你需要`source`一下生成的配置文件：

```bash
$ . ~/catkin_ws/devel/setup.bash
```

### 软件包依赖

一级依赖

```bash
$ rospack depends1 beginner_tutorials 
```

这些依赖关系存储在**package.xml**文件中

还有间接依赖 比如

```bash
$ rospack depends1 rospy
```

```bash
genpy
roscpp
rosgraph
rosgraph_msgs
roslib
std_msgs
```

使用`rospack`可以递归检测出所有嵌套的依赖包

```bash
$ rospack depends beginner_tutorials
cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy
```



### 更新rosdep时出现的问题

`$ rospack depends1 beginner_tutorials` 时 报错

` [rospack] Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'`

检查一下`$ rospack find rosdep` 

`[rospack] Error: package 'rosdep' not found` 是因为rosdep这个包没有装

于是`$ sudo apt install rospack-tools`

装好之后 `$ sudo rosdep init`

报错`sudo: rosdep: command not found`

再检查 `$ rospack find rosdep`

提示我们要这样安装 `Command 'rosdep' not found, but can be installed with:sudo apt install python3-rosdep2`

于是`$ sudo apt install python3-rosdep`

装好之后 `$ rosdep update`

```bash
ERROR: no sources directory exists on the system meaning rosdep has not yet been initialized.

Please initialize your rosdep with

	sudo rosdep init

```

跑`$ sudo rosdep init`

```bash
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
Recommended: please run

	rosdep update

```

再次 `$ rosdep update`

~~！！搞人心态的伟大的()GFW和DNS污染来了！！~~ 

```bash
reading in sources list data from /etc/ros/rosdep/sources.list.d
ERROR: error loading sources list:
	The read operation timed out

```

再尝试一下 `$ rosdep update`

这次多给了一些信息

```bash
Leading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
ERROR: error loading sources list:
	The read operation timed out

```

之后无论尝试多少遍 都是

```
reading in sources list data from /etc/ros/rosdep/sources.list.d
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml]:
	<urlopen error [Errno 111] Connection refused> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml)
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml]:
	<urlopen error [Errno 111] Connection refused> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml)
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml]:
	<urlopen error [Errno 111] Connection refused> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml)
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml]:
	<urlopen error [Errno 111] Connection refused> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml)
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml]:
	Failed to download target platform data for gbpdistro:
	<urlopen error [Errno 111] Connection refused>
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
ERROR: error loading sources list:
	<urlopen error <urlopen error [Errno 111] Connection refused> (https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml)>

```

接下来 我尝试了各种各样的解决办法，和GFW斗智斗勇

1. `$ sudo gedit /etc/resolv.conf`

   将原有的nameserver这一行注释，并添加以下两行：

   nameserver 8.8.8.8 #google域名服务器

   nameserver 8.8.4.4 #google域名服务器

   保存退出，执行

   `$ sudo  apt-get update`

2. 打开terminal，输入`$ sudo gedit /etc/host`
   在文件末尾添加`151.101.84.133 raw.githubusercontent.com`

3. 虚拟机上代理

   具体参考 https://blog.csdn.net/songchuwang1868/article/details/105173284/

4. 以上全都无效，实在没法，然后随便挑了个报错链接甩进浏览器发现都是可以访问的，那能不能手动下载之后更改一下rosdep update的更新脚本

   照这个思路 参考https://blog.csdn.net/super_sean/article/details/105433250

   主要是修改 `/etc/ros/rosdep/sources.list.d/20-default.list`文件和 `/usr/lib/python3/dist-packages/rosdistro/__init__.py`文件

   最后还是修改了DNS服务器 然后成功更新

   

## 自定义软件包

### 每个软件包里的package.xml

1. description

   ```xml
   <description>The beginner_tutorials package</description>
   ```

   想写啥写啥

2. maintainer

   ```xml
     <!-- One maintainer tag required, multiple allowed, one person per tag --> 
     <!-- Example:  -->
     <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
     <maintainer email="user@todo.todo">user</maintainer>
   ```

   让其他人联系到软件包的相关人员。至少需要填写一个维护者，但如果你想的话可以添加多个。除了在标签里面填写维护者的名字外，还应该在标签的`email`属性中填写电子邮件地址

3. license

   ```xml
     <!-- One license tag required, multiple allowed, one license per tag -->
     <!-- Commonly used license strings: -->
     <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
     <license>TODO</license>
   ```

   一些常见的开源许可协议有BSD、MIT、Boost Software License、GPLv2、GPLv3、LGPLv2.1和LGPLv3。你可以在[Open Source Initiative](http://opensource.org/licenses/alphabetical)中阅读其中的若干个许可协议的相关信息。对于本教程，我们使用BSD许可证，因为其他核心ROS组件已经在使用它

4. 依赖项

   这些依赖项分为`build_depend`、`buildtool_depend`、`run_depend`、`test_depend`

   有关这些标记的详细说明，请参考[Catkin Dependencies](http://wiki.ros.org/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies)相关的文档

   在之前的操作中，因为我们将`roscpp`、`rospy`和`std_msgs`作为[catkin_create_pkg](http://wiki.ros.org/catkin/commands/catkin_create_pkg)命令的参数，因此依赖关系如下所示

   ```xml
     <!-- The *_depend tags are used to specify dependencies -->
     <!-- Dependencies can be catkin packages or system dependencies -->
     <!-- Examples: -->
     <!-- Use build_depend for packages you need at compile time: -->
     <!--   <build_depend>genmsg</build_depend> -->
     <!-- Use buildtool_depend for build tool packages: -->
     <!--   <buildtool_depend>catkin</buildtool_depend> -->
     <!-- Use exec_depend for packages you need at runtime: -->
     <!--   <exec_depend>python-yaml</exec_depend> -->
     <!-- Use test_depend for packages you need only for testing: -->
     <!--   <test_depend>gtest</test_depend> -->
     <buildtool_depend>catkin</buildtool_depend>
     <build_depend>roscpp</build_depend>
     <build_depend>rospy</build_depend>
     <build_depend>std_msgs</build_depend>
   ```

   除了catkin中默认提供的`buildtool_depend`，所有我们列出的依赖包都已经被添加到`build_depend`标签中。在本例中，因为在编译和运行时都需要用到所有指定的依赖包，因此还要将每一个依赖包分别添加到`exec_depend`标签中：

   ```xml
     <buildtool_depend>catkin</buildtool_depend>
   
     <build_depend>roscpp</build_depend>
     <build_depend>rospy</build_depend>
     <build_depend>std_msgs</build_depend>
   
     <exec_depend>roscpp</exec_depend>
     <exec_depend>rospy</exec_depend>
     <exec_depend>std_msgs</exec_depend>
   ```

依赖项标签主要用来定义该功能包编译和执行时依赖的库，编译工具等，主要分为以下 6 类依赖：

1. 构建工具依赖项 `buildtool_depend`：指定构建该功能包使用的工具，默认使用 catkin
2. 构建依赖项 `<build_depend>`：指定编译功能包需要的依赖包
3. 构建导出依赖项 `<build_export_depend>`：指定对功能包构建库需要依赖哪些功能包
4. 执行依赖项 `exec_depend`：指定运行该功能包所依赖的其他功能包
5. 测试依赖项 `test_depend`：指定进行单元测试的依赖包
6. 文档工具依赖项 `doc_depend`：指定使用什么文档生成工具来为该功能包生成文档

**重要**：在 Format 2 语法中，使用 `<depend>` 标签来作为 `<build_depend>`、`build_export_depend` 和 `<exec_depend>` 这 3 个标签功能的组合，这是官方建议使用的标签：

```xml
<depend> roscpp </depend>

等价于以下 3 行依赖关系：

<build_depend> roscpp </build_depend>
<build_export_depend> roscpp </build_export_depend>
<exec_depend> roscpp </exec_depend>
```



## 开始编译软件包

只要安装了这个包的所有系统依赖项，就可以开始编译软件包了

**注意：**如果你是通过`apt`或者其它软件包管理器来安装ROS的，那么系统已经安装好了所有的依赖项。

先要设置环境 source一下

```bash
$ source /opt/ros/<distro>/setup.bash
```

[catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) 是一个命令行工具，它简化了标准catkin工作流程。你可以认为`catkin_make`是在标准CMake工作流程中依次调用了`cmake`和`make`。

```bash
# 在catkin工作空间下
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

（标准CMake流程）

```bash
# 在CMake工作空间下
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install  # （可选）
```

每个CMake项目都要单独进行这样的步骤。相反，多个catkin项目可以放在工作空间中一起构建，在工作空间中构建零到多个catkin软件包为以下工作流程：

```bash
# 在catkin工作空间下
$ catkin_make
$ catkin_make install  # （可选）
```

上述命令会构建`src`目录下的所有catkin项目

如果你的源代码不在默认位置（catkin_ws/src），比如说存放在了`my_src`中，那可以这样来使用catkin_make:

```bash
# 在catkin工作空间下
$ catkin_make --source my_src
$ catkin_make install --source my_src  # （可选）
```

现在来测试一下编译刚用`catkin_create_pkg`命令创建的软件包`beginner_tutorials`

```bash
$ cd ~/catkin_ws/
$ catkin_make
```

请注意，[catkin_make](http://wiki.ros.org/catkin/commands/catkin_make)首先输出它所使用到的每个**空间**所在的路径

需要注意的是，根据这些变量的默认值，有几个目录已经在catkin工作空间中自动生成了

```bash
$ ls
```

```bash
build
devel
src
```

`build` 目录是[构建空间](http://wiki.ros.org/catkin/workspaces#Build_Space)的默认位置，同时`cmake`和`make`也是在这里被调用来配置和构建你的软件包。而`devel`目录是[开发空间](http://wiki.ros.org/catkin/workspaces#Development_.28Devel.29_Space)的默认位置, 在安装软件包之前，这里可以存放可执行文件和库。



## ROS图

[计算图（Computation Graph）](http://wiki.ros.org/cn/ROS/Concepts#ROS.2Bi6F7l1b.2BXEJrIQ-)是一个由ROS进程组成的点对点网络，它们能够共同处理数据。ROS的基本计算图概念有节点（Nodes）、主节点（Master）、参数服务器（Parameter Server）、消息（Messages）、服务（Services）、话题（Topics）和袋（Bags），它们都以不同的方式向图（Graph）提供数据

- [节点（Nodes）](http://wiki.ros.org/Nodes)：节点是一个可执行文件，它可以通过ROS来与其他节点进行通信。
- [消息（Messages）](http://wiki.ros.org/Messages)：订阅或发布话题时所使用的ROS数据类型。
- [话题（Topics）](http://wiki.ros.org/Topics)：节点可以将消息*发布*到话题，或通过*订阅*话题来接收消息。
- [主节点（Master）](http://wiki.ros.org/Master)：ROS的命名服务，例如帮助节点发现彼此。
- [rosout](http://wiki.ros.org/rosout)：在ROS中相当于`stdout/stderr（标准输出/标准错误）`。
- [roscore](http://wiki.ros.org/roscore)：主节点 + rosout + [参数服务器](http://wiki.ros.org/Parameter Server)（会在以后介绍）。



### 节点

节点实际上只不过是ROS软件包中的一个可执行文件。ROS节点使用ROS[客户端库](http://wiki.ros.org/cn/Client Libraries)与其他节点通信。节点可以发布或订阅话题，也可以提供或使用[服务](http://wiki.ros.org/Services)。

**注：**节点是ROS中非常重要的一个概念，为了帮助初学者理解这个概念，这里举一个通俗的例子：

例如，咱们有一个机器人，和一个遥控器，那么这个机器人和遥控器开始工作后，就是两个节点。遥控器起到了下达指 令的作用；机器人负责监听遥控器下达的指令，完成相应动作。从这里我们可以看出，节点是一个能执行特定工作任 务的工作单元，并且能够相互通信，从而实现一个机器人系统整体的功能。在这里我们把遥控器和机器人简单定义为两个节点，实际上在机器人中根据控制器、传感器、执行机构等不同组成模块，还可以将其进一步细分为更多的节点，这个是根据用户编写的程序来定义的。）



### 客户端库

ROS客户端库可以让用不同编程语言编写的节点进行相互通信：

- rospy = Python客户端库
- roscpp = C++客户端库



### roscore

`roscore`是你在运行所有ROS程序前首先要运行的命令。

测试一下

```bash
$ roscore
```

输出了：

```bash
... logging to /home/zhaorui/.ros/log/4bb2ac16-2829-11ec-a8a5-cbad660fe4b9/roslaunch-ubuntu-39461.log # ~/.ros/log/9cf88ce4-b14d-11df-8a75-00251148e8cf/roslaunch-machine_name-13039.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:42801/ # http://machine_name:33919/ 参数服务器？
ros_comm version 1.15.13


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.13

NODES

auto-starting new master #主节点
process[master]: started with pid [39472]
ROS_MASTER_URI=http://ubuntu:11311/ # http://machine_name:11311/

setting /run_id to 4bb2ac16-2829-11ec-a8a5-cbad660fe4b9
process[rosout-1]: started with pid [39493]
started core service [/rosout] # rosout

```

如果`roscore`运行后没有初始化，很有可能是网络配置的问题。参见[网络配置 - 单机器配置](http://wiki.ros.org/ROS/NetworkSetup#Single_machine_configuration)。

如果`roscore`不能初始化并提示缺少权限，可能是因为`~/.ros`目录属于`root`用户（只有`root`用户才能访问），可以用以下命令递归地更改该目录的所有权：

```
$ sudo chown -R <your_username> ~/.ros
```



开一个新终端 用rosnode看看roscore运行时做了些什么

**注意：** 当打开一个新的终端时，环境将会重置，`~/.bashrc`文件将会生效。如果你在运行`rosnode`等命令时出现一些问题，那么可能需要将一些环境设置文件添加到`~/.bashrc`或手动`source`一下。

```bash
$ rosnode list #rosnode显示当前正在运行的ROS节点信息 list会列出活动的结点
```

```bash
/rosout
```

这表示当前只有一个节点在运行: [rosout](http://wiki.ros.org/rosout)。因为这个节点用于收集和记录节点的调试输出，所以它总是在运行的。

而`rosnode info`命令返回的是某个指定节点的信息。

```bash
$ rosnode info /rosout
```

```bash
--------------------------------------------------------------------------------
Node [/rosout]
Publications: 
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions: 
 * /rosout [unknown type]

Services: 
 * /rosout/get_loggers
 * /rosout/set_logger_level


contacting node http://ubuntu:41723/ ...
Pid: 39493
```

### rosrun

`rosrun`可以让你用包名直接运行软件包内的节点（而不需要知道包的路径）。

```bash
$ rosrun [package_name] [node_name]
```

测试一下，试着运行`turtlesim`包中的`turtlesim_node`

在一个**新**终端中：

```bash
$ rosrun turtlesim turtlesim_node
```

[![5PDTl8.png](https://z3.ax1x.com/2021/10/08/5PDTl8.png)](https://imgtu.com/i/5PDTl8)

此处的乌龟可能和你turtlesim窗口上的不同。别担心，实际上有[许多版本的turtle](http://wiki.ros.org/Distributions#Current_Distribution_Releases) ，而你的是个惊喜!（一个可爱的小彩蛋～）

再次用`$ rosnode list`查看：

```bash
/rosout
/turtlesim
```

ROS有一个强大的功能，就是你可以通过命令行重新分配名称。

关闭turtlesim窗口以停止节点（或回到`rosrun turtlesim`的终端并按`Ctrl+C`）。现在让我们重新运行它，但是这一次使用[重映射参数](http://wiki.ros.org/Remapping Arguments)来改变节点名称：

```bash
$ rosrun turtlesim turtlesim_node __name:=my_turtle
```

现在，如果我们回去使用`rosnode list`：

```bash
$ rosnode list
```

你会看到类似下面的输出信息：

```bash
/rosout
/my_turtle
```

再使用`$ rosnode ping my_turtle`  测试下它是否正常

```bash
rosnode: node is [/my_turtle]
pinging /my_turtle with a timeout of 3.0s
xmlrpc reply from http://ubuntu:42803/  time=0.548601ms
xmlrpc reply from http://ubuntu:42803/  time=0.550508ms
xmlrpc reply from http://ubuntu:42803/  time=0.810146ms
xmlrpc reply from http://ubuntu:42803/  time=1.718044ms
xmlrpc reply from http://ubuntu:42803/  time=0.644207ms
xmlrpc reply from http://ubuntu:42803/  time=0.636578ms
xmlrpc reply from http://ubuntu:42803/  time=0.543833ms
xmlrpc reply from http://ubuntu:42803/  time=0.494957ms
xmlrpc reply from http://ubuntu:42803/  time=0.744343ms
^Cping average: 0.743469ms
```



## ROS话题

首先确保roscore正在运行，只有一个roscore在运行就够了

开一个turtlesim

```bash
$ rosrun turtlesim turtlesim_node
```

### 通过键盘遥控turtle

开一个新终端

```bash
$ rosrun turtlesim turtle_teleop_key
```

```bash
[ INFO] 1254264546.878445000: Started node [/teleop_turtle], pid [5528], bound on [aqy], xmlrpc port [43918], tcpros port [55936], logging to [~/ros/ros/log/teleop_turtle_5528.log], using [real] time
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
```

用方向键来遥控乌龟吧，注意选中打开了turtle_teleop_key的终端以保证键盘输入能被捕获

[![5Py4YR.png](https://z3.ax1x.com/2021/10/08/5Py4YR.png)](https://imgtu.com/i/5Py4YR)



### ROS话题

#### 使用rqt_graph

`turtlesim_node`节点和`turtle_teleop_key`节点之间是通过一个ROS**话题**来相互通信的。`turtle_teleop_key`在话题上**发布**键盘按下的消息，`turtlesim`则**订阅**该话题以接收消息。让我们使用[rqt_graph](http://wiki.ros.org/rqt_graph)来显示当前运行的节点和话题

`rqt_graph`用动态的图显示了系统中正在发生的事情

打开一个**新**终端：

```bash
$ rosrun rqt_graph rqt_graph
```

可以看到

[![5PgWTg.png](https://z3.ax1x.com/2021/10/08/5PgWTg.png)](https://imgtu.com/i/5PgWTg)

如果把鼠标放在`/turtle1/cmd_vel`上方，相应的ROS节点（这里是蓝色和绿色）和话题（这里是红色）就会高亮显示。可以看到，`turtlesim_node`和`turtle_teleop_key`节点正通过一个名为`/turtle1/cmd_vel`的话题来相互通信。

![key.png](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key2.png)

箭头方向是从发消息一方到订阅话题的一方



#### rostopic

`rostopic`命令工具能让你获取ROS**话题**的信息

##### rostopic echo 

可以显示在某个话题上发布的数据

```bash
rostopic is a command-line tool for printing information about ROS Topics.

Commands:
        rostopic bw     display bandwidth used by topic
        rostopic delay  display delay of topic from timestamp in header
        rostopic echo   print messages to screen
        rostopic find   find topics by type
        rostopic hz     display publishing rate of topic    
        rostopic info   print information about active topic
        rostopic list   list active topics
        rostopic pub    publish data to topic
        rostopic type   print topic or field type

Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
```

```bash
rostopic echo [topic]
```

让我们看看由`turtle_teleop_key`节点发布的“指令、速度”数据。

**在ROS Hydro及更新版本中，**这些数据发布在`/turtle1/cmd_vel`话题上。打开一个**新**终端：

```
$ rostopic echo /turtle1/cmd_vel
```

**在ROS Hydro及更新版本中，**当按下向上键时，应该看到以下内容：

```bash
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
```

[![5PR7ZT.png](https://z3.ax1x.com/2021/10/08/5PR7ZT.png)](https://imgtu.com/i/5PR7ZT)

现在让我们再看一下`rqt_graph`。先按下左上角的刷新按钮以显示新节点。正如你所看到的，`rostopic echo`现在也**订阅**了`turtle1/cmd_vel`话题

##### rostpoic list

`rostopic list`能够列出当前已被订阅和发布的所有话题

```bash
Usage: rostopic list [/topic]

Options:
  -h, --help            show this help message and exit
  -b BAGFILE, --bag=BAGFILE
                        list topics in .bag file
  -v, --verbose         list full details about each topic
  -p                    list only publishers
  -s                    list only subscribers
```

试试`-verbose`参数

```bash
Published topics:
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /rosout [rosgraph_msgs/Log] 3 publishers
 * /turtle1/pose [turtlesim/Pose] 1 publisher
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher

Subscribed topics:
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /turtle1/cmd_vel [geometry_msgs/Twist] 2 subscribers
```



### ROS消息

话题的通信是通过节点间发送ROS**消息**实现的。为了使发布者（`turtle_teleop_key`）和订阅者（`turtulesim_node`）进行通信，发布者和订阅者必须发送和接收相同**类型**的消息。这意味着话题的**类型**是由发布在它上面消息的**类型**决定的。使用`rostopic type`命令可以查看发布在话题上的消息的**类型**。

#### rostopic type

```bash
rostopic type [topic]
```

```bash
$ rostopic type /turtle1/cmd_vel
```

得到

```bash
geometry_msgs/Twist
```

#### rosmsg

再用rosmsg查看消息的详细信息

```bash
$ rosmsg show geometry_msgs/Twist
```

```bash
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```



### rostopic 和 rosmsg 结合

#### rostopic pub

`rostopic pub`可以把数据发布到当前某个正在广播的话题上

```bash
Usage: rostopic pub [topic] [msg_type] [args]

Options:
  -h, --help            show this help message and exit
  -v                    print verbose output
  -r RATE, --rate=RATE  publishing rate (hz).  For -f and stdin input, this
                        defaults to 10.  Otherwise it is not set.
  -1, --once            publish one message and exit
  -f FILE, --file=FILE  read args from YAML file (Bagy)
  -l, --latch           enable latching for -f, -r and piped input.  This
                        latches the first message.
  -s, --substitute-keywords
                        When publishing with a rate, performs keyword ('now'
                        or 'auto') substitution for each message
  --use-rostime         use rostime for time stamps, else walltime is used
```

使用

```bash
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```

会发送一条消息`geometry_msgs/Twist`给所有订阅了`/turtle1/cmd_vel`话题的人让它们以2.0大小的线速度和1.8大小的角速度移动

`--`这一选项（两个破折号）用来告诉选项解析器，表明之后的参数都不是选项。如果参数前有破折号（-）比如负数，那么这是必需的。

如前所述，一个turtlesim/Velocity消息有两个浮点型元素：`linear`和`angular`。在本例中，`'[2.0, 0.0, 0.0]'`表示`linear`的值为`x=2.0`, `y=0.0`, `z=0.0`，而`'[0.0, 0.0, 1.8]'`是说`angular`的值为`x=0.0`, `y=0.0`, `z=1.8`。这些参数实际上使用的是YAML语法，在[YAML命令行文档](http://wiki.ros.org/ROS/YAMLCommandLine)中有描述。

```yaml
'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
```

你可能已经注意到turtle已经停止移动了。这是因为turtle需要一个稳定的频率为1Hz的指令流才能保持移动状态。我们可以使用`rostopic pub -r`命令来发布源源不断的命令：

```bash
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

这将以1 Hz的速度发布velocity指令到velocity话题上。

[![5P4hkT.png](https://z3.ax1x.com/2021/10/08/5P4hkT.png)](https://imgtu.com/i/5P4hkT)

我们还可以看一下`rqt_graph`中的情形。按左上角的刷新按钮，可以看到rostopic pub节点（此处为红色）正在与rostopic echo节点（此处为绿色）进行通信：

[![5P50D1.md.png](https://z3.ax1x.com/2021/10/08/5P50D1.md.png)](https://imgtu.com/i/5P50D1)

```bash
rostopic echo /turtle1/pose
```

```bash

---
x: 8.443611145019531
y: 8.190336227416992
theta: -3.082719564437866
linear_velocity: 2.0
angular_velocity: -1.7999999523162842
---
x: 8.411625862121582
y: 8.189373970031738
theta: -3.1115195751190186
linear_velocity: 2.0
angular_velocity: -1.7999999523162842
---
x: 8.379626274108887
y: 8.189332962036133
theta: -3.140319585800171
linear_velocity: 2.0
angular_velocity: -1.7999999523162842
---
x: 8.347638130187988
y: 8.190214157104492
theta: 3.1140658855438232
linear_velocity: 2.0
angular_velocity: -1.7999999523162842
---
...
```

#### rostopic hz

`rostopic hz`报告数据发布的速率

```bash
rostopic hz [topic]
```

我们看一下`turtlesim_node`发布`/turtle/pose`得有多快：

```bash
$ rostopic hz /turtle1/pose
```

```bash
average rate: 62.500
        min: 0.015s max: 0.017s std dev: 0.00049s window: 63
average rate: 62.496
        min: 0.014s max: 0.019s std dev: 0.00071s window: 125
average rate: 62.480
        min: 0.014s max: 0.019s std dev: 0.00068s window: 188
average rate: 62.484
        min: 0.014s max: 0.019s std dev: 0.00065s window: 250
average rate: 62.478
        min: 0.014s max: 0.019s std dev: 0.00065s window: 313
average rate: 62.493
        min: 0.014s max: 0.019s std dev: 0.00066s window: 375
```

现在我们可以知道了turtlesim正以大约60Hz的频率发布有关乌龟的数据。我们也可以结合`rostopic type`和`rosmsg show`命令来获取关于某个话题的更深层次信息

我们也可以结合`rostopic type`和`rosmsg show`命令来获取关于某个话题的更深层次信息

```bash
$ rostopic type /turtle1/cmd_vel | rosmsg show
```



### 使用rqt_plot

`rqt_plot`命令可以在滚动时间图上显示发布到某个话题上的数据。这里我们将使用`rqt_plot`命令来绘制正被发布到`/turtle1/pose`话题上的数据。首先，在一个**新终端**中输入：

```bash
$ rosrun rqt_plot rqt_plot
```

这会弹出一个新窗口，可以在左上角的文本框里面添加任何想要绘制的话题。在里面输入`/turtle1/pose/x`后，之前不能按下的加号按钮将会变亮。按一下该按钮，并对`/turtle1/pose/y`重复相同的过程。现在你会在图中看到turtle的x-y位置。

[![5iCEVK.md.png](https://z3.ax1x.com/2021/10/09/5iCEVK.md.png)](https://imgtu.com/i/5iCEVK)

按下减号按钮会显示一组菜单可以让你在图中隐藏指定的话题。现在隐藏掉你刚才添加的话题并添加`/turtle1/pose/theta`，你会看到如下所示的图：

[![5iCGa8.md.png](https://z3.ax1x.com/2021/10/09/5iCGa8.md.png)](https://imgtu.com/i/5iCGa8)



## ROS服务和参数

### ROS服务

[服务（Services）](http://wiki.ros.org/Services)是节点之间通讯的另一种方式。服务允许节点发送一个**请求（request）**并获得一个**响应（response）**

### 使用rosservice

`rosservice`可以很容易地通过服务附加到ROS的C/S框架上。`rosservice`有许多可用于服务的命令，如下所示：

```bash
rosservice list         输出活跃服务的信息
rosservice call         用给定的参数调用服务
rosservice type         输出服务的类型
rosservice find         按服务的类型查找服务
rosservice uri          输出服务的ROSRPC uri
```

#### rosservice list

```bash
$ rosservice list
```

```bash
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/teleop_turtle/get_loggers
/teleop_turtle/set_logger_level
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```

list命令显示为turtlesim节点提供了9个服务：`reset`, `clear`, `spawn`, `kill`, `turtle1/set_pen`, `/turtle1/teleport_absolute`, `/turtle1/teleport_relative`, `turtlesim/get_loggers`, `turtlesim/set_logger_level`。同时还有两个与`rosout`节点有关的服务：`/rosout/get_loggers`和`/rosout/set_logger_level`

#### rosservice type

```bash
rosservice type [service]
```

我们用`rosservice type`进一步查看clear的服务

```bash
$ rosservice type /clear
```

```bash
std_srvs/Empty
```

服务的类型为`empty`（空），这表明**调用**这个服务时不需要参数（即，它在发出**请求**时不发送数据，在接收**响应**时也不接收数据）

#### rosservice call

接着来调用`clear`服务

```bash
rosservice call [service] [args]
```

因为服务的类型为empty，所以进行无参数调用：

```bash
$ rosservice call /clear
```

跟想象的一样，它清除了`turtlesim_node`背景上的轨迹。

[![5iP1SJ.png](https://z3.ax1x.com/2021/10/09/5iP1SJ.png)](https://imgtu.com/i/5iP1SJ)

再来看看服务有参数的情况。查看spawn服务的信息

```bash
$ rosservice type /spawn | rossrv show
```

```
float32 x
float32 y
float32 theta
string name
---
string name
```

这个服务能让我们可以在给定的位置和角度生成一只新的乌龟。`name`字段是可选的，这里我们不设具体的名字，让turtlesim自动创建一个

```bash
$ rosservice call /spawn 2 2 0.2 ""
```

调用返回了乌龟名字

```
name: "turtle2"
```

[![5iPDld.png](https://z3.ax1x.com/2021/10/09/5iPDld.png)](https://imgtu.com/i/5iPDld)



### 使用rosparam

`rosparam`能让我们在ROS[参数服务器（Parameter Server）](http://wiki.ros.org/Parameter Server)上存储和操作数据。参数服务器能够存储整型（integer）、浮点（float）、布尔（boolean）、字典（dictionaries）和列表（list）等数据类型。rosparam使用YAML标记语言的语法。一般而言，YAML的表述很自然：`1`是整型，`1.0`是浮点型，`one`是字符串，`true`是布尔型，`[1, 2, 3]`是整型组成的列表，`{a: b, c: d}`是字典

`rosparam`有很多命令可以用来操作参数：

```bash
rosparam set            设置参数
rosparam get            获取参数
rosparam load           从文件中加载参数
rosparam dump           向文件中转储参数
rosparam delete         删除参数
rosparam list           列出参数名
```

#### rosparam list

先看看参数服务器上有哪些参数

```bash
$ rosparam list
```

```
/rosdistro
/roslaunch/uris/host_ubuntu__46451
/rosversion
/run_id
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

#### rosparam set 和 rosparam get

```bash
rosparam set [param_name]
rosparam get [param_name]
```

试着用`rosparam set`来改变一个参数, 修改背景颜色的红色通道值：

```bash
$ rosparam set /turtlesim/background_r 150
```

上述指令修改了参数的值，现在我们需要调用clear服务使得参数的修改能生效：

```bash
$ rosservice call /clear
```

[![5iiP76.png](https://z3.ax1x.com/2021/10/09/5iiP76.png)](https://imgtu.com/i/5iiP76)

接着，查看参数服务器上其它参数的值，比如获取背景绿色通道值：

```bash
$ rosparam get /turtlesim/background_g
```

```
86
```

也可以用`rosparam get /`来显示参数服务器上的所有内容：

```bash
$ rosparam get /
```

```
rosdistro: 'noetic

  '
roslaunch:
  uris:
    host_ubuntu__46451: http://ubuntu:46451/
rosversion: '1.15.13

  '
run_id: a304f43c-2852-11ec-93d5-61bec7b79451
turtlesim:
  background_b: 255
  background_g: 86
  background_r: 150
```

#### rosparam dump 和 rosparam load

可能希望将刚刚获得的参数存储在文件中，以方便下次加载就需要用到

```bash
rosparam dump [file_name] [namespace]
rosparam load [file_name] [namespace]
```

在这里，我们将所有的参数写入`params.yaml`文件：

```bash
$ rosparam dump params.yaml
```

你甚至可以将yaml文件重载入新的命名空间，例如`copy_turtle`：

```bash
$ rosparam load params.yaml copy_turtle
$ rosparam get /copy_turtle/turtlesim/background_b
```

```bash
255
```



## rqt_console 和 roslaunch

### rqt_console

`rqt_console`连接到了ROS的日志框架，以显示节点的输出信息。`rqt_logger_level`允许我们在节点运行时改变输出信息的详细级别，包括`Debug`、`Info`、`Warn`和`Error`

现在让我们来看一下turtlesim在rqt_console中输出的信息，同时在使用turtlesim时切换`rqt_logger_level`中的日志级别。在启动turtlesim之前先在两个**新**终端中运行`rqt_console`和`rqt_logger_level`

[![5E0yQ0.png](D:\我的公众号\5E0yQ0.png)](https://imgtu.com/i/5E0yQ0)

[![5E0qeO.png](D:\我的公众号\5E0qeO.png)](https://imgtu.com/i/5E0qeO)

现在让我们在另一个**新**终端中启动turtlesim：

```bash
$ rosrun turtlesim turtlesim_node
```

因为默认的日志级别是Info，所以你会看到turtlesim启动后发布的所有信息，如下图所示：

[![5EDrrV.md.png](D:\我的公众号\5EDrrV.md.png)](https://imgtu.com/i/5EDrrV)



### 日志记录器级别

日志级别的优先级按以下顺序排列：

```
Fatal （致命）
Error （错误）
Warn  （警告）
Info  （信息）
Debug （调试）
```

`Fatal`是最高优先级，`Debug`是最低优先级。通过设置日志级别，你可以获得所有优先级级别，或只是更高级别的消息。比如，将日志级别设为`Warn`时，你会得到`Warn`、`Error`和`Fatal`这三个等级的日志消息。

现在按`Ctrl+C`退出turtlesim节点。接下来我们将使用`roslaunch`来启动多个turtlesim节点和一个模仿者节点，来让一个乌龟模仿另一个乌龟。



### roslaunch

`roslaunch`可以用来启动定义在`launch（启动）`文件中的节点

```bash
$ roslaunch [package] [filename.launch]
```

先切换到我们之前[创建](http://wiki.ros.org/cn/ROS/Tutorials/CreatingPackage)和[构建](http://wiki.ros.org/cn/ROS/Tutorials/BuildingPackages)的`beginner_tutorials`软件包目录下：

```
$ roscd beginner_tutorials
```

如果`roscd`提示类似于`roscd: No such package/stack 'beginner_tutorials'`的话，你需要按照[创建catkin工作空间](http://wiki.ros.org/cn/catkin/Tutorials/create_a_workspace)后面的步骤使环境变量生效：

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscd beginner_tutorials
```

然后创建一个`launch`目录：

```
$ mkdir launch
$ cd launch
```

**注意：**存放launch文件的目录不一定非要命名为`launch`，事实上都不用非得放在目录中，`roslaunch`命令会自动查找经过的包并检测可用的启动文件。然而，这种推荐的标准做法被认为是“最佳实践”。

#### launch文件

现在一起创建一个名为`turtlemimic.launch`的launch文件并复制粘贴以下内容进去：

```xml
<launch>  <!-- launch标签开头表明launch文件 -->

  <group ns="turtlesim1"> 
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

```xml
 <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>
```

此处我们创建了两个分组，并以命名空间（namespace）标签来区分，其中一个名为`turtulesim1`，另一个名为`turtlesim2`，两个分组中都有相同的名为`sim`的turtlesim节点。这样可以让我们同时启动两个turtlesim模拟器，而不会产生命名冲突

```xml
  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>
```

在这里我们启动模仿节点，话题的输入和输出分别重命名为`turtlesim1`和`turtlesim2`，这样就可以让turtlesim2模仿turtlesim1了。

#### 使用roslaunch

```bash
$ roslaunch beginner_tutorials turtlemimic.launch
```

现在将会有两个turtlesim被启动，然后我们在一个**新**终端中使用`rostopic`命令发送：

```bash
$ rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

可以看到两个turtlesims同时开始移动，虽然发布命令只发送给了turtlesim1

[![5E6Ue0.md.png](D:\我的公众号\5E6Ue0.md.png)](https://imgtu.com/i/5E6Ue0)

我们还可以用[rqt_graph](http://wiki.ros.org/rqt_graph)来更好地理解launch文件所做的事情。运行[rqt](http://wiki.ros.org/rqt)并在主窗口中选择*Plugins > Introspection > Node Graph*：

```bash
$ rqt
```

或者直接运行：

```bash
$ rqt_graph
```

[<img src="D:\我的公众号\5Ec8AK.md.png" alt="5Ec8AK.md.png" style="zoom: 200%;" />](https://imgtu.com/i/5Ec8AK)

我们算是已经学会了rqt_console和roslaunch命令的使用，接下来我们开始学习[使用rosed](http://wiki.ros.org/cn/ROS/Tutorials/UsingRosEd)了解ROS的编辑器选择。现在你可以按`Ctrl+C`退出所有turtlesims节点了，因为在接下来的教程中你不会再用到它们



## rosed

### 使用rosed

利用它可以直接通过软件包名编辑包中的文件，而无需键入完整路径

需要安装vim

如果文件名在包中不是唯一的，则菜单将提示你选择要编辑哪个文件

rosed命令可用tab补全文件名

\<tab>\<tab>可以查看包中所有文件

rosed默认的编辑器是vim。你可以**把下面这行加到`~/.bashrc`文件中**来更改默认编辑器：比如更改为nano

```bash
export EDITOR='nano -w'
```

***注意：*** *.bashrc文件的改变只对新打开的终端有效。之前已经打开的终端不受更改环境变量的影响。*

打开一个新的终端，看看是否定义了`EDITOR`变量：

```bash
$ echo $EDITOR
```

```
nano -w
```



## 创建消息和服务

### msg和srv

- [msg](http://wiki.ros.org/msg)（消息）：msg文件就是文本文件，用于描述ROS消息的字段。它们用于为不同编程语言编写的消息生成源代码。
- [srv](http://wiki.ros.org/srv)（服务）：一个srv文件描述一个服务。它由两部分组成：请求（request）和响应（response）。

msg文件存放在软件包的`msg`目录下，srv文件则存放在`srv`目录下

msg文件就是简单的文本文件，每行都有一个字段类型和字段名称。可以使用的类型为：

- int8, int16, int32, int64 (以及 uint*)
- float32, float64
- string
- time, duration
- 其他msg文件
- variable-length array[] 和 fixed-length array[C]

ROS中还有一个特殊的数据类型：`Header`，它含有时间戳和ROS中广泛使用的坐标帧信息。在msg文件的第一行经常可以看到`Header header`。

下面是一个msg文件的样例，它使用了Header，string，和其他另外两个消息的类型：

```
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
```

srv文件和msg文件一样，只是它们包含两个部分：请求和响应。这两部分用一条`---`线隔开。下面是一个srv文件的示例：

```
int64 A
int64 B
---
int64 Sum
```

在上面的例子中，`A`和`B`是请求, `Sum`是响应。

### 使用msg

#### 创建msg

```bash
$ roscd beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```

上面是最简单的示例，`.msg`文件只有一行。当然，你可以通过添加更多元素（每行一个）来创建一个更复杂的文件，如下所示：

```
string first_name
string last_name
uint8 age
uint32 score
```

不过还有关键的一步：我们要确保msg文件能被转换为C++、Python和其他语言的源代码。

打开`package.xml`, 确保它包含以下两行且没有被[注释](http://www.htmlhelp.com/reference/wilbur/misc/comment.html)。如果没有，添加进去：

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

注意，在构建时，其实只需要`message_generation`，而在运行时，我们只需要`message_runtime`。

在你喜欢的文本编辑器中打开`CMakeLists.txt`文件（[rosed](http://wiki.ros.org/cn/ROS/Tutorials/UsingRosEd)是一个不错的选择）。

在`CMakeLists.txt`文件中，为已经存在里面的`find_package`调用添加`message_generation`依赖项，这样就能生成消息了。直接将`message_generation`添加到`COMPONENTS`列表中即可，如下所示：

```
# 不要直接复制这一大段，只需将message_generation加在括号闭合前即可
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

你可能注意到了，有时即使没有使用全部依赖项调用find_package，项目也可以构建。这是因为catkin把你所有的项目整合在了一起，因此如果之前的项目调用了find_package，你的依赖关系也被配置成了一样的值。但是，忘记调用意味着你的项目在单独构建时很容易崩溃。

还要确保导出消息的运行时依赖关系：

```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

找到如下代码块：

```
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```

删除`#`符号来取消注释，然后将`Message*.msg`替换为你的.msg文件名，就像下边这样：

```
add_message_files(
  FILES
  Num.msg
)
```

手动添加.msg文件后，我们要确保CMake知道何时需要重新配置项目。

现在必须确保`generate_messages()`函数被调用：取消注释

```
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
```



#### 使用rosmsg

以上就是创建消息的所有步骤。让我们通过`rosmsg show`命令看看ROS能否识别它。

用法：

```bash
$ rosmsg show [message type]
```

示例：

```bash
$ rosmsg show beginner_tutorials/Num
```

你会看到：

```
int64 num
```

在上面的例子中，消息类型包含两部分：

- `beginner_tutorials` -- 定义消息的软件包
- `Num` -- 消息的名称`Num`

如果不记得msg在哪个包中，也可以省略包名称。尝试：

```bash
$ rosmsg show Num
```



### 使用srv

#### 创建srv

让我们使用之前创建的包再来创建服务：

```bash
$ roscd beginner_tutorials
$ mkdir srv
```

我们将从另一个包复制现有的srv定义，而不是手动创建新的srv。`roscp`是一个实用的命令行工具，用于将文件从一个包复制到另一个包。

用法：

```bash
$ roscp [package_name] [file_to_copy_path] [copy_path]
```

现在我们可以从[rospy_tutorials](http://wiki.ros.org/rospy_tutorials)包中复制一个服务：

```bash
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

之后也同样要确保msg文件能被转换为C++、Python和其他语言的源代码

和上面创建msg中的过程一样

（别被名字迷惑，`message_generation`对`msg`和`srv`都适用）

此外，你也需要像之前对消息那样在package.xml中修改服务字段，因此请看上面描述的所需附加依赖项。

```
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
```

删除`#`符号来取消注释，然后将`Service*.srv`替换为你的.srv文件名，就像下边这样：

```
add_service_files(
  FILES
  AddTwoInts.srv
)
```



#### 使用rossrv

让我们通过`rossrv show`命令看看ROS能否识别它

```bash
$ rossrv show <service type>
```

```bash
$ rossrv show beginner_tutorials/AddTwoInts
```



### msg和srv的一般步骤

如果没做过上面的教程，请先修改下`CMakeLists.txt`：

```
# generate_messages(
#   DEPENDENCIES
# #  std_msgs  # Or other packages containing msgs
# )
```

取消注释，然后添加任意你的消息用到的包含`.msg`文件的软件包（本例中为`std_msgs`），如下所示：

```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

现在我们已经创建了一些新消息，所以需要重新`make`一下软件包：

```bash
# In your catkin workspace
$ roscd beginner_tutorials
$ cd ../..
$ catkin_make
$ cd -
```

`msg`目录中的任何`.msg`文件都将生成所有支持语言的代码。C++消息的头文件将生成在`~/catkin_ws/devel/include/beginner_tutorials/`。Python脚本将创建在`~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg`。而Lisp文件则出现在`~/catkin_ws/devel/share/common-lisp/ros/beginner_tutorials/msg/`。

类似地，`srv`目录中的任何`.srv`文件都将生成支持语言的代码。对于C++，头文件将生成在消息的头文件的同一目录中。对于Python和Lisp，会在`msg`目录旁边的`srv`目录中。

消息格式的完整规范在[消息描述语言](http://wiki.ros.org/ROS/Message_Description_Language)中。

如果你正在构建使用新消息的C++节点，则还需要声明节点和消息之间的依赖关系



## 编写简单的发布者和订阅者

### 编写发布者

“节点”是连接到ROS网络的可执行文件。在这里，我们将创建`talker`（发布者）节点，该节点将不断广播消息。

将目录切换到之前的教程中[创建](http://wiki.ros.org/cn/ROS/Tutorials/CreatingPackage)的beginner_tutorials包中：

```bash
$ roscd beginner_tutorials
```

首先让我们创建一个`scripts`目录来存放我们的Python脚本：

```bash
$ mkdir scripts
$ cd scripts
```

然后下载示例脚本[talker.py](https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/001_talker_listener/talker.py)放到`scripts`目录中并给执行权限：

```
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

```bash
$ wget https://raw.github.com/ros/ros_tutorials/noetic-devel/rospy_tutorials/001_talker_listener/talker.py # 若遇到网络问题，请打开上面文件的链接并复制文本内容到talker.py文件中
$ chmod +x talker.py
```

先不要运行它。你可以通过下面的命令查看和编辑这个文件。

```bash
$ rosed beginner_tutorials talker.py 
```

将以下内容添加到`CMakeLists.txt`文件。这样可以确保正确安装Python脚本，并使用合适的Python解释器。

```
catkin_install_python(PROGRAMS scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

现在，让我们把代码分解。

```python
#!/usr/bin/env python
```

每个Python ROS节点的最开头都有这个声明。第一行确保脚本作为Python脚本执行。

```python
import rospy
from std_msgs.msg import String
```

如果要编写ROS节点，则需要导入`rospy`。`std_msgs.msg`的导入则是为了使我们能够重用`std_msgs/String`消息类型（即一个简单的字符串容器）来发布。

```python
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
```

这部分代码定义了talker与其他ROS部分的接口。`pub = rospy.Publisher("chatter", String, queue_size=10)`声明该节点正在使用`String`消息类型发布到`chatter`话题。这里的`String`实际上是`std_msgs.msg.String`类。`queue_size`参数是在ROS Hydro及更新版本中新增的，用于在订阅者接收消息的速度不够快的情况下，限制排队的消息数量。

下一行的`rospy.init_node(NAME, ...)`非常重要，因为它把该节点的名称告诉了rospy——只有rospy掌握了这一信息后，才会开始与ROS主节点进行通信。在本例中，你的节点将使用`talker`名称。注意：名称必须是[基本名称](http://wiki.ros.org/Names)，例如不能包含任何斜杠`/`。

`anonymous = True`会让名称末尾添加随机数，来确保节点具有唯一的名称

```python
rate = rospy.Rate(10) # 10hz
```

此行创建一个`Rate`对象`rate`。借助其方法`sleep()`，它提供了一种方便的方法，来以你想要的速率循环。它的参数是`10`，即表示希望它每秒循环10次（只要我们的处理时间不超过十分之一秒）！

```python
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
```

这个循环是一个相当标准的rospy结构：检查`rospy.is_shutdown()`标志，然后执行代码逻辑。你必须查看`is_shutdown()`以检查程序是否应该退出（例如有`Ctrl+C`或其他）。在本例中，代码逻辑即对`public .publish(hello_str)`的调用，它将一个字符串发布到`chatter`话题。循环的部分还调用了`rate.sleep()`，它在循环中可以用刚刚好的睡眠时间维持期望的速率。

你可能也见过`rospy.sleep()`，它和`time.sleep()`类似，不同的是前者还能用于模拟时间

此循环还调用了`rospy.loginfo(str)`，它有3个任务：打印消息到屏幕上；把消息写入节点的日志文件中；写入[rosout](http://wiki.ros.org/rosout)。[rosout](http://wiki.ros.org/rosout)是一个方便的调试工具：您可以使用[rqt_console](http://wiki.ros.org/rqt_console)来拉取消息，而不必在控制台窗口找你节点的输出

`std_msgs.msg.String`是一个非常简单的消息类型，那更复杂的类型怎么发布呢？一般的经验法则是*构造函数参数的顺序与`.msg`文件中的顺序相同*。也可以不传入任何参数，直接初始化字段，例如：

```python
msg = String()
msg.data = str
```

或者可以只初始化某些字段，并将其余字段保留为默认值：

```python
String(data=str)
```

```python
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

除了标准的Python `__main__`检查，它还会捕获一个`rospy.ROSInterruptException`异常，当按下`Ctrl+C`或节点因其他原因关闭时，这一异常就会被`rospy.sleep()`和`rospy.Rate.sleep()`抛出。引发此异常的原因是你不会意外地在`sleep()`之后继续执行代码。



### 编写订阅者

下载示例脚本[listener.py](https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/001_talker_listener/listener.py)放到`scripts`目录中并给执行权限：

```
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

```bash
$ roscd beginner_tutorials/scripts/
$ wget https://raw.github.com/ros/ros_tutorials/noetic-devel/rospy_tutorials/001_talker_listener/listener.py # 若遇到网络问题，请打开上面文件的链接并复制文本内容到listener.py文件中
$ chmod +x listener.py
```

然后，编辑你`CMakeLists.txt`中的`catkin_install_python()`调用，如下所示：

```
catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

`listener.py`的代码类似于`talker.py`，只不过我们为订阅消息引入了一种新的基于回调的机制

```python
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
```

这声明你的节点订阅了`chatter`话题，类型是`std_msgs.msgs.String`。当接收到新消息时，`callback`函数被调用，**消息作为第一个参数**。

我们还稍微更改了对`rospy.init_node()`的调用。我们添加了`anonymous=True`关键字参数。ROS要求每个节点都有一个唯一的名称，如果出现具有相同名称的节点，则会与前一个节点发生冲突，这样一来，出现故障的节点很容易地被踢出网络。`anonymous=True`标志会告诉`rospy`为节点生成唯一的名称，这样就很容易可以有多个`listener.py`一起运行。

最后再补充一下，`rospy.spin()`只是不让你的节点退出，直到节点被明确关闭。与roscpp不同，rospy.spin()不影响订阅者回调函数，因为它们有自己的线程。

我们使用CMake作为构建系统。是的，即使是Python节点也必须使用它。这是为了确保能为创建的消息和服务自动生成Python代码。

回到catkin工作空间，然后运行`catkin_make`：

```bash
$ cd ~/catkin_ws
$ catkin_make
```



## 运行及测试发布者和订阅者

确保roscore已经开启：

```bash
$ roscore
```

**catkin specific** 如果使用catkin，在运行你的程序前，请确保你在调用`catkin_make`后已经`source`过工作空间的`setup.*sh`文件：

```bash
# 在catkin工作空间中
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```

上一教程中，我们制作了一个叫做`talker`的发布者，让我们运行它：

```bash
$ rosrun beginner_tutorials talker.py   # (Python)
```

你会看到：

```
[INFO] [1633921588.206784]: hello world 1633921588.206672
[INFO] [1633921588.307886]: hello world 1633921588.3076153
[INFO] [1633921588.407568]: hello world 1633921588.4073088
[INFO] [1633921588.508212]: hello world 1633921588.5078938
[INFO] [1633921588.607821]: hello world 1633921588.6077063
[INFO] [1633921588.707382]: hello world 1633921588.70725
[INFO] [1633921588.807208]: hello world 1633921588.8070707
[INFO] [1633921588.908052]: hello world 1633921588.9079382
```

发布者节点已启动并运行。现在我们需要一个订阅者以接收来自发布者的消息。

上一教程中，我们也制作了一个叫做`listener`的订阅者，让我们运行它：

```bash
$ rosrun beginner_tutorials listener.py  # (Python) 
```

你会看到：

```
[INFO] [1633921725.009392]: /listener_36602_1633921724646I heard hello world 1633921725.0077126
[INFO] [1633921725.111057]: /listener_36602_1633921724646I heard hello world 1633921725.1078975
[INFO] [1633921725.208652]: /listener_36602_1633921724646I heard hello world 1633921725.2072344
[INFO] [1633921725.308458]: /listener_36602_1633921724646I heard hello world 1633921725.3068624
[INFO] [1633921725.409068]: /listener_36602_1633921724646I heard hello world 1633921725.4074502
```

完成后，按`Ctrl+C`停止listener和talker



## 编写简单的服务和客户端

### 编写服务节点

这里，我们将创建简单的服务（Service）节点`add_two_ints_server`，该节点将接收两个整数，并返回它们的和。

将当前目录切换到之前的[教程](http://wiki.ros.org/cn/ROS/Tutorials/CreatingPackage)中创建的beginner_tutorials包中：

```bash
$ roscd beginner_tutorials
```

请确保你已经按照[之前教程](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)中的指示创建了本教程中需要的服务`AddTwoInts.srv`

在beginner_tutorials包中创建`scripts/add_two_ints_server.py`文件并粘贴以下内容进去：

```python
#!/usr/bin/env python

from __future__ import print_function

from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```

别忘了给节点执行权限：

```bash
$ chmod +x scripts/add_two_ints_server.py
```

然后将以下内容添加到`CMakeLists.txt`文件。这样可以确保正确安装Python脚本，并使用合适的Python解释器。

```
catkin_install_python(PROGRAMS scripts/add_two_ints_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

现在，让我们把代码分解。

使用[rospy](http://wiki.ros.org/rospy)编写服务的难度非常小。我们使用`init_node()`声明我们的节点，然后再声明我们的服务

```
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
```

这声明了一个名为`add_two_ints`的新服务，其服务类型为`AddTwoInts`。所有的请求（request）都传递给了`handle_add_two_ints`函数。`handle_add_two_ints`被`AddTwoIntsRequest`的实例调用，返回`AddTwoIntsResponse`实例

就像订阅者中的例子一样，`rospy.spin()`可以防止代码在服务关闭之前退出



### 编写客户结点

在beginner_tutorials包中创建`scripts/add_two_ints_client.py`文件并粘贴以下内容进去：

```python
#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
```

别忘了给节点执行权限：

```bash
$ chmod +x scripts/add_two_ints_client.py
```

然后，在你的`CMakeLists.txt`中编辑`catkin_install_python()`调用，就像这样：

```
catkin_install_python(PROGRAMS scripts/add_two_ints_server.py scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

现在，让我们把代码分解。

客户端（用来调用服务）的代码也很简单。对于客户端来说不需要调用`init_node()`。我们首先调用：

```python
    rospy.wait_for_service('add_two_ints')
```

这是一种很方便的方法，可以让在`add_two_ints`服务可用之前一直阻塞。

```python
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
```

这里我们为服务的调用创建了一个句柄（handle）

```python
        resp1 = add_two_ints(x, y)
        return resp1.sum
```

然后我们可以使用这个句柄，就像普通的函数一样调用它。

因为我们已经将服务的类型声明为`AddTwoInts`，它会为你生成`AddTwoIntsRequest`对象 (you're free to pass in your own instead)。如果调用失败，`rospy.ServiceException`将会抛出，所以你应该弄一个合适的`try/except`部分



### 小疑问：import是如何工作的

```py
|-- src
|   `-- beginner_tutorials
|       |-- scripts
|       |   `-- add_two_ints_server.py
|       `-- srv
|           `-- AddTwoInts.srv
|-- build
|-- devel // this is where your modules are imported from
```

When you build the package using catkin_make, `catkin` generates the relevant python files for your service type defined in .srv file and puts them under catkin_ws/devel/lib/**your-python-version**/dist-packages/**package-name**/srv.

If your workspace is sourced, catkin_ws/devel/lib/**your-python-version**/dist-packages/ is already added to your PYTHONPATH and that is how you are able to import them successfully.

In case of the tutorial package that you are using, imports may work even when you haven't sourced your current catkin-directory, if you have the binaries of the tutorials installed. This way the python modules reside under /opt/ros/**ros-version**/lib/**your-python-version**/dist-packages/ and that is again part of the PYTHONPATH. (If ROS env is available)



### 构建结点

我们使用CMake作为构建系统。是的，即使是Python节点也必须使用它。这是为了确保能为[创建的消息和服务](http://wiki.ros.org/cn/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv)自动生成Python代码。

切换当前目录到你的catkin工作空间，然后运行`catkin_make`：

```bash
# 在你的catkin工作空间中
$ cd ~/catkin_ws
$ catkin_make
```



## 运行及测试服务和客户端

运行服务端：

```bash
$ rosrun beginner_tutorials add_two_ints_server.py  # (Python)
```

你会看到：

```
Ready to add two ints.
```

运行客户端：

```bash
$ rosrun beginner_tutorials add_two_ints_client.py 1 3  # (Python)
```

```
Requesting 1+3
1 + 3 = 4
```



## 录制和回放数据

正在运行的ROS系统中的数据记录到一个bag文件中，然后通过回放这些数据来来重现相似的运行过程



### 录制数据

首先，在**不同**的终端中分别执行：

*终端1：*

```bash
$ roscore
```

*终端2：*

```bash
$ rosrun turtlesim turtlesim_node
```

*终端3：*

```bash
$ rosrun turtlesim turtle_teleop_key
```



#### 录制所有发布的话题

首先让我们来检查一下当前系统中发布的所有话题。打开一个新终端：

```bash
$ rostopic list -v
```

输出类似于：

```
Published topics:
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /rosout [rosgraph_msgs/Log] 2 publishers
 * /turtle1/pose [turtlesim/Pose] 1 publisher
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher

Subscribed topics:
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
```

已发布主题的列表是唯一可能被记录在数据日志文件中的消息类型，因为只有发布的消息才能被录制。由`teleop_turtle`发布的`/turtle1/cmd_vel`话题是指令消息，作为turtlesim进程的输入。消息`/turtle1/color_sensor`和`/turtle1/pose`是turtlesim发布的输出消息。

现在我们将记录发布的数据。打开一个新终端：

```bash
$ mkdir ~/bagfiles
$ cd ~/bagfiles
$ rosbag record -a
```

在运行`rosbag record`的窗口中按`Ctrl+C`以退出。现在查看`~/bagfiles`目录中的内容，你应该会看到一个以年份、日期和时间开头且扩展名是`.bag`的文件。这就是传说中的袋文件，它包含`rosbag record`运行期间由任何节点发布的所有话题



### 检查并回放bag文件

现在我们已经使用`rosbag record`命令录制了一个bag文件，接下来我们可以使用`rosbag info`查看它的内容，或用`rosbag play`命令回放。首先我们来看看袋子里记录了什么。我们可以执行**info**命令检查bag文件的内容而不回放它。在bag文件所在的目录下执行以下命令：

```bash
$ rosbag info <your bagfile>
```

你会看到：

```
path:        2014-12-10-20-08-34.bag
version:     2.0
duration:    1:38s (98s)
start:       Dec 10 2014 20:08:35.83 (1418270915.83)
end:         Dec 10 2014 20:10:14.38 (1418271014.38)
size:        865.0 KB
messages:    12471
compression: none [1/1 chunks]
types:       geometry_msgs/Twist [9f195f881246fdfa2798d1d3eebca84a]
             rosgraph_msgs/Log   [acffd30cd6b6de30f120938c17c593fb]
             turtlesim/Color     [353891e354491c51aabe32df673fb446]
             turtlesim/Pose      [863b248d5016ca62ea2e895ae5265cf9]
topics:      /rosout                    4 msgs    : rosgraph_msgs/Log   (2 connections)
             /turtle1/cmd_vel         169 msgs    : geometry_msgs/Twist
             /turtle1/color_sensor   6149 msgs    : turtlesim/Color
             /turtle1/pose           6149 msgs    : turtlesim/Pose
```

这些信息告诉你bag文件中所包含话题的名称、类型和消息数量。我们可以看到，在之前使用**rostopic**命令查看到的五个已公告的话题中，其实只有四个在我们录制期间发布了消息。因为我们带-a参数选项运行**rosbag record**命令时系统会录制下所有节点发布的所有消息。

下一步是回放bag文件以再现系统运行过程。首先用`Ctrl+C`杀死之前运行的turtle_teleop_key，但让turtlesim继续运行。在终端中bag文件所在目录下运行以下命令：

```bash
$ rosbag play <your bagfile>
```

在这个窗口中你应该会立即看到如下类似信息：

```
[ INFO] [1418271315.162885976]: Opening 2014-12-10-20-08-34.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
```

默认模式下，**rosbag play**命令在公告每条消息后会等待一小段时间（0.2秒）才真正开始发布bag文件中的内容。等待一段时间是为了可以通知订阅者，消息已经公告且数据可能会马上到来。如果**rosbag play**在公告消息后立即发布，订阅者可能会接收不到几条最先发布的消息。等待时间可以通过`-d`选项来指定。

最终`/turtle1/cmd_vel`话题将会被发布，同时在turtuelsim中乌龟应该会像之前用turtle_teleop_key控制它那样开始移动。从运行**rosbag play**到乌龟开始移动时所经历时间应该近似等于之前在本教程开始部分运行**rosbag record**后到开始按下键盘发出控制命令时所经历时间。你可以通过`-s`参数选项让**rosbag play**不从bag文件的开头开始，而是从某个指定的时间开始。最后一个可能比较有趣的参数选项是`-r`选项，它允许你通过设定一个参数来改变消息发布速率。如果执行：

```bash
$ rosbag play -r 2 <your bagfile>
```

你应该会看到乌龟的运动轨迹有点不同了，这时的轨迹应该是相当于当你以两倍的速度通过按键发布控制命令时产生的轨迹



### 录制数据子集

当运行一个复杂的系统时，比如PR2软件套装，会有几百个话题被发布，有些话题亦会发布大量数据（比如包含摄像头图像流的话题）。在这种系统中，将包含所有话题的日志写入一个bag文件到磁盘通常是不切实际的。**rosbag record**命令支持只录制特定的话题到bag文件中，这样就可以只录制用户感兴趣的话题。

如果还有turtlesim节点在运行，先退出他们，然后重新启动键盘控制节点相关的启动文件：

```bash
$ rosrun turtlesim turtlesim_node 
$ rosrun turtlesim turtle_teleop_key
```

在bag文件所在目录下执行以下命令：

```bash
$ rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```

上述命令中的`-O`参数告诉**rosbag record**将数据记录到名为`subset.bag`的文件中，而后面的`topic`参数告诉**rosbag record**只能订阅这两个指定的话题。然后通过键盘控制乌龟随意移动几秒钟，最后按`Ctrl+C`退出**rosbag record**命令。

现在看看bag文件中的内容（`rosbag info subset.bag`）。你应该会看到如下类似信息，里面只包含指定的话题: 

```bash
path:        subset.bag
version:     2.0
duration:    25.8s
start:       Oct 11 2021 04:37:40.55 (1633952260.55)
end:         Oct 11 2021 04:38:06.34 (1633952286.34)
size:        133.5 KB
messages:    1653
compression: none [1/1 chunks]
types:       geometry_msgs/Twist [9f195f881246fdfa2798d1d3eebca84a]
             turtlesim/Pose      [863b248d5016ca62ea2e895ae5265cf9]
topics:      /turtle1/cmd_vel     40 msgs    : geometry_msgs/Twist
             turtle1/pose       1613 msgs    : turtlesim/Pose
```



### rosbag录制和回放的局限性

在上一小节中，你可能已经注意到了乌龟的路径可能并没有完全地映射到原先通过键盘控制时产生的路径上——整体形状应该是差不多的，但没有完全一样。这是因为turtlesim的移动路径对系统定时精度的变化非常敏感。rosbag受制于其本身的性能无法完全复制录制时的系统运行行为，rosplay也一样。对于像turtlesim这样的节点，当处理消息的过程中系统定时发生极小变化时也会使其行为发生微妙变化，用户不应该期望能够完美地模仿系统行为。



## 从bag文件中读取消息

从bag文件中读取所需话题的消息的两种方法，以及`ros_readbagfile`脚本的使用

首先，需要一个bag，可从

```bash
wget https://open-source-webviz-ui.s3.amazonaws.com/demo.bag
```

下载一个。

*请注意，下面所有的命令中，前面都有一个`time`，这样做可以同时输出执行每个命令花费的时间，而且有时这些命令需要很长时间，因此使用`time`命令了解给定命令所需的时间是有必要的。如果您不想使用它，可以放心删除下面任何命令中的`time`*

### immediate回放并在多个终端查看输出

你需要知道你想从bag文件中读取的**准确**话题名。那让我们看看袋子里有什么。在任何终端中用这个命令，来手动检查所有已发布的话题，以及向每个话题发布了多少消息：

```
time rosbag info demo.bag  
# 或者你已经知道话题名称的话：
time rosbag info mybag.bag | grep -E "(topic1|topic2|topic3)"
```

你会看到：

```
$ time rosbag info demo.bag  
path:        demo.bag
version:     2.0
duration:    20.0s
start:       Mar 21 2017 19:37:58.00 (1490150278.00)
end:         Mar 21 2017 19:38:17.00 (1490150298.00)
size:        696.2 MB
messages:    5390
compression: none [600/600 chunks]
types:       bond/Status                      [eacc84bf5d65b6777d4c50f463dfb9c8]
                diagnostic_msgs/DiagnosticArray  [60810da900de1dd6ddd437c3503511da]
                diagnostic_msgs/DiagnosticStatus [d0ce08bc6e5ba34c7754f563a9cabaf1]
                nav_msgs/Odometry                [cd5e73d190d741a2f92e81eda573aca7]
                radar_driver/RadarTracks         [6a2de2f790cb8bb0e149d45d297462f8]
                sensor_msgs/Image                [060021388200f6f0f447d0fcd9c64743]
                sensor_msgs/NavSatFix            [2d3a8cd499b9b4a0249fb98fd05cfa48]
                sensor_msgs/PointCloud2          [1158d486dd51d683ce2f1be655c3c181]
                sensor_msgs/Range                [c005c34273dc426c67a020a87bc24148]
                sensor_msgs/TimeReference        [fded64a0265108ba86c3d38fb11c0c16]
                tf2_msgs/TFMessage               [94810edda583a504dfda3829e70d7eec]
                velodyne_msgs/VelodyneScan       [50804fc9533a0e579e6322c04ae70566]
   topics:      /diagnostics                      140 msgs    : diagnostic_msgs/DiagnosticArray 
                /diagnostics_agg                   40 msgs    : diagnostic_msgs/DiagnosticArray 
                /diagnostics_toplevel_state        40 msgs    : diagnostic_msgs/DiagnosticStatus
                /gps/fix                          146 msgs    : sensor_msgs/NavSatFix           
                /gps/rtkfix                       200 msgs    : nav_msgs/Odometry               
                /gps/time                         192 msgs    : sensor_msgs/TimeReference       
                /image_raw                        600 msgs    : sensor_msgs/Image               
                /obs1/gps/fix                      30 msgs    : sensor_msgs/NavSatFix           
                /obs1/gps/rtkfix                  200 msgs    : nav_msgs/Odometry               
                /obs1/gps/time                    136 msgs    : sensor_msgs/TimeReference       
                /radar/points                     400 msgs    : sensor_msgs/PointCloud2         
                /radar/range                      400 msgs    : sensor_msgs/Range               
                /radar/tracks                     400 msgs    : radar_driver/RadarTracks        
                /tf                              1986 msgs    : tf2_msgs/TFMessage              
                /velodyne_nodelet_manager/bond     80 msgs    : bond/Status                     
                /velodyne_packets                 200 msgs    : velodyne_msgs/VelodyneScan      
                /velodyne_points                  200 msgs    : sensor_msgs/PointCloud2

   real    0m1.003s
   user    0m0.620s
   sys 0m0.283s
```

可以看到，有30条消息发布在`/obs1/gps/fix`话题上，有40条消息发布在`/diagnostics_agg`话题上。让我们把这些提取出来。

在终端1（比如本终端）中，启动roscore，这样可以运行必需的ROS主节点：

```
roscore
```

打开另一个终端订阅`/obs1/gps/fix`话题并复读该话题上发布的所有内容，同时用`tee`命令转储到一个yaml格式的文件中以便之后查看：

```
rostopic echo /obs1/gps/fix | tee topic1.yaml
```

你会看到：

```
$ rostopic echo /obs1/gps/fix | tee topic1.yaml
WARNING: topic [/obs1/gps/fix] does not appear to be published yet
```

再打开一个新终端，订阅另一个话题`/diagnostics_agg`。

```
rostopic echo /diagnostics_agg | tee topic2.yaml
```

你会看到：

```
$ rostopic echo /diagnostics_agg | tee topic2.yaml
WARNING: topic [/diagnostics_agg] does not appear to be published yet
```

对其他你感兴趣的话题重复这一步骤，每个话题必须有自己的终端。

再打开另一个新终端来回放bag文件。这一次我们将尽可能快地回放bag文件（使用`--immediate`选项），**只**会发布我们感兴趣的话题。格式如下：

```
time rosbag play --immediate demo.bag --topics /topic1 /topic2 /topic3 /topicN
```

本例中，命令如下：

   ```
   time rosbag play --immediate demo.bag --topics /obs1/gps/fix /diagnostics_agg
   ```

你会看到：

   ```
   $ time rosbag play --immediate demo.bag --topics /obs1/gps/fix /diagnostics_agg
   [ INFO] [1591916465.758724557]: Opening demo.bag
   
   Waiting 0.2 seconds after advertising topics... done.
   
   Hit space to toggle paused, or 's' to step.
    [RUNNING]  Bag Time: 1490150297.770734   Duration: 19.703405 / 19.703405               
   Done.
   
   real  0m1.570s
   user  0m0.663s
   sys 0m0.394s
   ```

完成！现在看一下你的两个终端，每个终端都订阅了一个话题，每个话题类型的所有消息用YAML格式输出，每条消息之间用`---`分割。用你喜欢的文本编辑器（最好支持YAML的语法高亮，例如Visual Studio Code）来查看文件中的消息。例如，`topic1.yaml`中的最后两条消息是这样的：

```
---
header: 
  seq: 4027
  stamp: 
    secs: 1490150296
    nsecs:  66947432
  frame_id: "gps"
status: 
  status: 0
  service: 1
latitude: 37.4008017844
longitude: -122.108119889
altitude: -6.4380177824
position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
position_covariance_type: 0
---
header: 
  seq: 4028
  stamp: 
    secs: 1490150297
    nsecs: 744347249
  frame_id: "gps"
status: 
  status: 0
  service: 1
latitude: 37.4007565466
longitude: -122.108159482
altitude: -6.35130467023
position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
position_covariance_type: 0
---
```

如果由于一些原因某个`rostopic`进程丢失了消息，可以使用`Ctrl+C`终止该进程，然后重新启动它，并再次调用`rosbag play`命令。

### 使用ros_readbagfile脚本轻松提取感兴趣的话题

下载并安装`ros_readbag.py`：

```
# Download the file
wget https://raw.githubusercontent.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles/master/useful_scripts/ros_readbagfile.py
# Make it executable
chmod +x ros_readbagfile.py
# Ensure you have the ~/bin directory for personal binaries
mkdir -p ~/bin
# Move this executable script into that directory as `ros_readbagfile`, so that it will
# be available as that command
mv ros_readbagfile.py ~/bin/ros_readbagfile
# Re-source your ~/.bashrc file to ensure ~/bin is in your PATH, so you can use this
# new `ros_readbagfile` command you just installed
. ~/.bashrc
```

通过`rosbag info`命令确定要从bag文件中读取的**准确**话题名，如上面方法1的第一步所示。

然后使用`ros_readbagfile`，大体格式如下：

```
ros_readbagfile <mybagfile.bag> [topic1] [topic2] [topic3] [...]
```

要阅读上面方法1中显示的相同消息，请使用：

```
time ros_readbagfile demo.bag /obs1/gps/fix /diagnostics_agg | tee topics.yaml
```

就是这样！你会看到它快速打印出所有70条信息。以下是终端输出的最后部分：

```
        key: "Early diagnostic update count:"
        value: "0"
      - 
        key: "Zero seen diagnostic update count:"
        value: "0"
=======================================
topic:           /obs1/gps/fix
msg #:           30
timestamp (sec): 1490150297.770734310
- - -
header: 
  seq: 4028
  stamp: 
    secs: 1490150297
    nsecs: 744347249
  frame_id: "gps"
status: 
  status: 0
  service: 1
latitude: 37.4007565466
longitude: -122.108159482
altitude: -6.35130467023
position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
position_covariance_type: 0
=======================================
Total messages found = 70.
DONE.

real  0m2.897s
user  0m2.457s
sys 0m0.355s
```

现在用你喜欢的文本编辑器打开**topics.yaml**，看看它从bag文件中提取的所有消息。

*请注意，尽管我给了这个文件一个“.yaml”扩展名，但并不代表所有部分都是正确的YAML格式。相反，尽管文件中存储的每条消息都是有效的YAML语法，但是消息之间的标题和行分隔符（例如`=====`）不是有效的。请记住这一点，避免试图将输出解析为YAML。如果你愿意，也可以很容易地修改`ros_readbagfile`这个Python脚本来删除这些非YAML特性。*

### 为什么ros_readbagfile而不rostopic echo -b

因为`rostopic`**极其地**慢！ 举个例子，就算在高配计算机（4核8线程的奔腾i7和m.2固态硬盘）上运行这个命令，也需要**11.5分钟**才能读取一个18GB的bag文件！

```
time rostopic echo -b large_bag_file.bag /topic1
```

而用`ros_readbagfile`脚本，在相同计算机上只要花费**1分钟37秒**就能读取同样的话题和18GB的bag文件！因此`ros_readbagfile`比`rostopic`快了11.5/(1+37/60) = **大约7倍**！

```
time ros_readbagfile large_bag_file.bag /topic1
```

因为`rostopic`一次只能读取**单个话题**，而`ros_readbagfile`可以同时读取**任意多的话题**！

```
ros_readbagfile <mybagfile.bag> [topic1] [topic2] [topic3] [...] [topic1000]
```



## roswtf检查错误

**在开始本教程之前，请确保`roscore`没有运行。**

对于Linux，您可以通过以下方式检查roscore是否在运行（如果看到类似这样的一行包含`rosmaster`，这是roscore的一部分，则说明roscore正在运行）

```
$ ps -ef | grep -i rosmaster
00:00:00 /usr/bin/python /opt/ros/kinetic/bin/rosmaster 
```



### 安装检查

```
$ roscd rosmaster
$ roswtf
```

你应该会看到（各种详细的输出信息）：

```
Package: rosmaster
================================================================================
Static checks summary:

No errors or warnings
================================================================================

ROS Master does not appear to be running.
Online graph checks will not be run.
ROS_MASTER_URI is [http://localhost:11311]
```

如果你的ROS安装没问题，应该会看到类似上面的输出信息，它的含义是：

- `Package: rosmaster`：[roswtf](http://wiki.ros.org/roswtf)使用当前目录中的任何内容来确定其执行的检查。这个输出告诉我们是在包`rosmaster`的目录中启动了`roswtf`。
- `Static checks summary`：它会报告任何关于文件系统或非运行时（比如无需roscore的场景）的问题。本例显示我们没有错误。
- `ROS Master does not appear to be running.`：`roscore`未在运行。`roswtf`不会做任何ROS在线检查。

### 在线检查

下一步，我们需要启动一个[Master](http://wiki.ros.org/Master)，所以要在新终端启动`roscore`，然后继续。

现在按照相同的顺序再次运行以下命令：

```
$ roscd
$ roswtf
```

你应该会看到：

```
No package or stack in context
======================================================
Static checks summary:

No errors or warnings
======================================================
Beginning tests of your ROS graph. These may take awhile...
analyzing graph...
... done analyzing graph
running graph rules...
... done running graph rules

Online checks summary:

Found 1 warning(s).
Warnings are things that may be just fine, but are sometimes at fault

WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```

`roscore`已经运行，`roswtf`刚刚做了一些ROS图的在线检查。根据您运行的ROS节点的数量，这可能需要很长时间才能完成。如你所见，这一次它产生了一个警告：

```
WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```

这一次，`roscd`在没有参数的情况下运行，这可能会带到一个没有ROS包的目录，因此我们看到了一条消息：`No package or stack in context`。

既然`roscore`已经运行了所以`roswtf`做了一些运行时检查。检查过程的长短取决于正在运行的ROS节点数量，可能会花费很长时间才能完成。正如你看到的，这一次出现了警告：

```
WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```

`roswtf`发出警告，[rosout](http://wiki.ros.org/rosout)节点订阅了一个没有节点向其发布的话题。在本例中，这正是所期望看到的，因为除了`roscore`没有任何其它节点在运行，所以我们可以忽略该警告。



### 错误

`roswtf`会对一些系统中看起来异常但可能是正常的运行情况发出警告。也会对确实有问题的情况报告错误。

接下来我们给`ROS_PACKAGE_PATH`环境变量设置一个*bad*值，并退出`roscore`以简化检查输出信息。

```
$ roscd
$ ROS_PACKAGE_PATH=bad:$ROS_PACKAGE_PATH roswtf
```

这次我们会看到：

```
Stack: ros
================================================================================
Static checks summary:

Found 1 error(s).

ERROR Not all paths in ROS_PACKAGE_PATH [bad] point to an existing directory: 
 * bad

================================================================================

Cannot communicate with master, ignoring graph checks
```

正如你看到的，`roswtf`发现了一个有关`ROS_PACKAGE_PATH`设置的错误。

`roswtf`还可以发现很多其它类型的问题。如果你发现自己被构建或通信的问题难住了，可以尝试运行`roswtf`看能否为你指明正确的方向。

