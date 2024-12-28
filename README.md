## Prerequisites
### Download docker
On Linux (or WSL)
```bash
sudo curl -L "https://github.com/docker/compose/releases/download/$(curl -s https://api.github.com/repos/docker/compose/releases/latest | jq -r .tag_name)/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
# verify installation
docker-compose --version
# python dependencies
pip install --upgrade setuptools
```

## Simulation (local testing)
### step 1: prepare docker image
```bash
cd ./docker/ros2_dev
sudo docker build -t ros2_dev_image .
```
### step 2: run docker container
```bash
sudo docker run -it --rm \
  -v path/to/ur/repo:/workspace \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  ros2_dev_image bash
ls /workspace # after entering the container make sure ur files are there
```
### step 3: build project
```bash
rm -rf build log install # ensures clean build
colcon build --symlink-install
source /opt/ros/humble/setup.bash 
```
### step 4: launch nodes
launching nodes individually
```bash
ros2 run control_node control_node
ros2 run camera_node camera_node
ros2 run object_detection_node object_detection_node
```
launching the system together
```bash
# move launch file to install file
cp ./launch/ml_tasks_launch.py ./install/control_node/share/control_node
# launch camera and object detection node together in terminal 1
ros2 launch control_node ml_tasks_launch.py
# open a new terminal and get the list of running docker containers
sudo docker ps
# enter the container with the name of the container
sudo docker exec -it container_name bash
# launch control_node in terminal 2
ros2 run control_node control_node
```


## Raspberry Pi Deployment
### step 1: connect to raspberry pi via ssh
```bash
ssh your_ssh_address
```
### step 2: prepare docker image
```bash
cd ./docker/ros2_deploy
docker build -t ros2_deploy_image .
```
### step 3: run docker container
### step 4: build project
### step 5: launch nodes
