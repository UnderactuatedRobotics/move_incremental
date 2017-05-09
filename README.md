## move_incremental
ROS move_base plugin that implements the D* Lite algorithm, for MIT 16.412 Cognitive Robotics Grand Challenge.
For more details about D* Lite, please check [[S. Koenig et al.- **D* Lite** -*aaai2002*]](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)

### Developer Note

We are using [Vincent Driessen's branching model](http://nvie.com/posts/a-successful-git-branching-model/) for git development workflow.

In a nutshell, **develop** branch is where several members' branches merge together. Individual developer should branch off, create feature/_your_feature_ branch, develop and then finish by merging it back to develop branch.  
 
[git-flow library of git subcommands](http://jeffkreeftmeijer.com/2010/why-arent-you-using-git-flow/) makes this just as easy as your daily git commands.

Installation of git-flow on Linuxes can be found [here](https://github.com/nvie/gitflow/wiki/Linux).

It's okay that you don't want to use git-flow: just manually create your feature branch and work, don't directly work on development branch. Just don't forget to turn off fast-forward to preserve your feature branch history by typing: `git merge --no-ff develop`.

### Compile Guide

First create your local catkin workspace, please follow [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Then, open your bash file (~/.bashrc) with your favorite editor (in my case, I use vim, so I type in `vim ~/.bashrc`)

And append the following line to the .bashrc file

`source /home/student/managed/catkin_ws/devel/setup.bash`
(so you don't have to source it every time you compile the code)

Then, 

`cd ~/catkin_ws/src` 

and 

`git clone https://github.com/UnderactuatedRobotics/move_incremental.git`

And finally we can build it:

`cd ~/catkin_ws` and `catkin_make`

### Simulation Launch Guide

In terminal, type:

`roslaunch move_incremental grand_challenge_sim.launch`

then Gazebo simulator will come out with summit-x in a predefined map.

To drive it around, open another terminal and type:

`rosrun cogrob achieve-sim-goals`
