# docker-poppy-ceril

Install git LFS

`$ git lfs install`

Navigate to the poppy robot docker directory

`$ cd docker/poppy`

Build the image for the poppy robot by running the script

`$ chmod u+x poppy_docker_build.sh`

`$ ./poppy_docker_build.sh`

Enable the xhost authentication by running the following command. 

`$ xhost +`

The command disables xhost authentications. This command is required to run the xterm from the container. This command will be replaced with a much safer option in the future.

Run the docker container script. 

`$ chmod u+x poppy_docker_run.sh`

`$ ./poppy_docker_run.sh`

A bash session(session 1) is launched. Run roscore

'# roscore'

Open a new bash session(session 2) in the same container from a new terminal

`$ docker exec -it poppy_ceril bash`

From inside the container, navigate to the poppy robot catkin workspace and build the package

`# source ros_entrypoint.sh`

`# cd poppy_ws`

`# catkin_make`

Source the setup.bash file for the poppy package and launch the poppy application

`# source devel/setup.bash`

`# roslaunch poppy_application poppy_app.launch`

If it's working, you should see the poppy robot launched in the Gazebo window and the joint states and joint position printed in the terminal of session 2.

Kill the ros application by hitting Ctrl+C key

quit the container by executing the following command in session 1:

`# exit`


