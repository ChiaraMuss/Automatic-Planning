
# to build

 sudo docker build --rm  --tag ros-humble . --file DockerFile

# to run

 sudo docker run -v /tmp/.X11-unix/:/tmp/.X11-unix/ --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --network=host --name ubuntu_bash --env="DISPLAY" --rm -i -t ros-humble bash

# Or

 sudo docker run -v /tmp/.X11-unix/:/tmp/.X11-unix/ --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --network=host --name ubuntu_bash --env="DISPLAY" --rm -i -t docker.io/library/ros-humble:latest bash

## For the plansys setup see the plansys2commands.md