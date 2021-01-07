# ver1.1.0-alpha

### Development Environment
1. Ubuntu 18.04 LTS

2. ROS Melodic


### Referenced Code
I used __iiwa_tool__ package of [_iiwa_stack_examples_][link1] and revised it to suit my project.
To use this package, [_iiwa_stack_][link2] also needs to be present on the system to make this package work.

[link1]: https://github.com/SalvoVirga/iiwa_stack_examples
[link2]: https://github.com/IFL-CAMP/iiwa_stack

### How to install
Installing method of _iiwa_stack_ is [here][link3].

[link3]:https://github.com/IFL-CAMP/iiwa_stack/wiki/roscore_setup

In my case,
 
    $ mkdir iiwa_stack_ws && cd iiwa_stack_ws && mkdir src
    $ cd src
    $ catkin_init_workspace
    $ git clone https://github.com/IFL-CAMP/iiwa_stack.git iiwa_stack
    $ rosdep install --from-paths src --ignore-src -r -y
    $ cd ..
    $ catkin build
    $ source devel/setup.bash

and then install _iiwa_stack_examples_.

    $ cd src
    $ git clone https://github.com/SalvoVirga/iiwa_stack_examples.git iiwa_stack_examples
    
### How to set ROS environment setting

    $ gedit ~/.bashrc
    
---
#Set ROS Melodic  
source /opt/ros/melodic/setup.bash    
source ~/iiwa_stack_ws/devel/setup.bash   

#Set ROS Network  
export ROS_HOSTNAME=xxx.xxx.xxx.xxx  
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311  

---

    $ source ~/.bashrc
    
With upper setting, you don't have to type $ source devel/setup.bash everytime when you open new terminal.
