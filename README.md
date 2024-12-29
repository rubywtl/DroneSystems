## Prerequisites
### WSL and Docker
Use WSL2 for a local Linux environment because it's integrated well in VSCode. Any Ubuntu is good. For **WSL (not Linux OS)** specifically, install Docker application on the Windows side for easier dev:
1. Install Docker Desktop: [Download Docker Desktop](https://www.docker.com/products/docker-desktop/)
2. Enable WSL integration in Docker Desktop settings.
3. Test Docker in your WSL terminal:

   ```bash
   docker --version
   ```

WSL distros are default installed to your OS drive, and moving it to another SSD for example could be a little tricky...

### Download docker-compose
On Linux (or WSL)
```bash
# install jq if not already
sudo apt install jq
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
  -v ~/<path to repo>:/workspace \
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
