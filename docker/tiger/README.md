# docker-tiger-ceril

Install git LFS

`$ git lfs install`

Navigate to the tiger robot docker directory
`$ cd docker/tiger`

Build the image for the tiger robot by running the script
`$ chmod u+x tiger_docker_build.sh`
`$ ./tiger_docker_build.sh`

Run the docker container script. 
`$ chmod u+x tiger_docker_run.sh`
`$ ./tiger_docker_run.sh`
A bash session(session 1) is launched. Run roscore
'# roscore'

Open a new bash session(session 2) in the same container from a new terminal
`$ docker exec -it tiger-ceril bash`

From inside the container, navigate to the tiger robot catkin workspace and build the package
`# source ros_entrypoint.sh`
`# cd catkin_ws`
`# catkin_make`

Source the setup.bash file for the tiger package and launch the tiger application
`# source devel/setup.bash`
`# roslaunch tiger_application tiger_app.launch`

If it's working, you should see the tiger robot launched in the Gazebo window and the joint states and joint position printed in the terminal of session 2.

Kill the ros application by hitting Ctrl+C key

quit the container by executing the following command in session 1:
`# exit`
