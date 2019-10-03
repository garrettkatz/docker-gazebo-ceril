# docker-gazebo-ceril

Install git LFS

`$ git lfs install`

Build the image: In the top-level of the repository run

`$ docker build -f ./docker/Dockerfile -t="docker-gazebo-ceril" .`

Run the docker container
`$ docker run -d -v="/tmp/.gazebo/:/root/.gazebo/" --rm --name=docker-gazebo-ceril docker-gazebo-ceril`

Open a new bash session in the same container
`$ docker exec -it docker-gazebo-ceril bash`

From inside the container, try running a shell script:

`# sh /src/test.sh`

If it's working, you should see CERIL's toy_example output and also a log of the pendulum recording


quit the container:

`# exit`
