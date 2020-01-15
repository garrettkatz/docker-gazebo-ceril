# docker-poppy-ceril

Install git LFS

`$ git lfs install`

Navigate to the poppy robot docker directory

`$ cd docker/poppy`

Build the image for the poppy robot by running the script

`$ chmod u+x poppy_docker_build.sh`

`$ ./poppy_docker_build.sh`

This file does several things:
1. Builds a docker image with ROS installed and a gui user
2. Runs a container in detached state named poppy_ceril
3. Executes the src/finish_build.sh script in the container to do two more things:
a. Assign the host user uid/gid to the gui user in the container to enable X11 support
b. Give the gui user permissions and environment needed for ROS in the container

The container poppy_ceril can be stopped and then restarted later

Execute a gui user bash session (session 1) in the container: 

`$ chmod u+x poppy_docker_run.sh`
`$ ./poppy_docker_run.sh`

In session 1, run roscore:

`$ roscore`

Open a new bash session(session 2) in the same container from a new host terminal

`$ ./poppy_docker_run.sh`

In session 2, navigate to the poppy robot catkin workspace, and build the package:

`# cd catkin_ws`

`# catkin_make`

Source the setup.bash file for the poppy package and launch the poppy application

`# source devel/setup.bash`

`# roslaunch poppy_application poppy_app.launch`

If it's working, you should see the poppy robot launched in the Gazebo window and the joint states and joint position printed in the terminal of session 2.

Kill the ros applications in each session by hitting Ctrl+C key

Exit both sessions with this command:

`# exit`

Stop the container if desired:

`$ docker stop poppy_ceril`

Later it can be restarted:

`$ docker start poppy_ceril`

