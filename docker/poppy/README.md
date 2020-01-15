# docker-poppy-ceril

Install git LFS

`$ git lfs install`

Navigate to the poppy robot docker directory

`$ cd docker/poppy`

Build the image for the poppy robot by running the script

`$ chmod u+x poppy_docker_build.sh`

`$ ./poppy_docker_build.sh`

This build includes X11 and user configuration to support GUIs

Run the docker container script. 

`$ chmod u+x poppy_docker_run.sh`

`$ ./poppy_docker_run.sh`

This script passes in current host UID/GID as environment variables in the container.  Then it runs `startup.sh` inside the container to configure UID/GID for X server access.  Then it configures the gui user to enable ROS usage.  Finally it launches a bash session as the gui user (session 1).  In that session, source the ROS entrypoint and run roscore:

`$ source ros_entrypoint.sh`
`$ roscore`

Open a new bash session(session 2) in the same container from a new terminal

`$ docker exec -it poppy_ceril /src/startup.sh`

From inside the container, source the entrypoint, navigate to the poppy robot catkin workspace, and build the package:

`# source ros_entrypoint.sh`

`# cd catkin_ws`

`# catkin_make`

Source the setup.bash file for the poppy package and launch the poppy application

`# source devel/setup.bash`

`# roslaunch poppy_application poppy_app.launch`

If it's working, you should see the poppy robot launched in the Gazebo window and the joint states and joint position printed in the terminal of session 2.

Kill the ros application by hitting Ctrl+C key

quit the container by executing the following command in session 1:

`# exit`


