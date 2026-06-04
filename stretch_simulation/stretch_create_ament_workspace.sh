#!/bin/bash

set -e

if [[ $EUID = 0 ]] && [[ -z "$DOCKER_BUILD" ]]; then
   echo "Please run this script without sudo."
   exit 1
fi


LOCALROBOT_NAME="stretch-se4-local"
sudo mkdir -p /etc/hello-robot
echo "HELLO_FLEET_ID=$LOCALROBOT_NAME" | sudo tee /etc/hello-robot/hello-robot.conf

. /etc/hello-robot/hello-robot.conf
echo "###########################################"
echo "NEW INSTALLATION OF USER SOFTWARE"
echo "###########################################"
echo "Update $HOME/.bashrc dotfile..."
echo "" >> $HOME/.bashrc
echo "######################" >> $HOME/.bashrc
echo "# STRETCH BASHRC SETUP" >> $HOME/.bashrc
echo "######################" >> $HOME/.bashrc
echo "export HELLO_FLEET_PATH=${HOME}/stretch_user" >> $HOME/.bashrc
echo "export HELLO_FLEET_ID=${HELLO_FLEET_ID}">> $HOME/.bashrc
echo "export PATH=\${PATH}:$HOME/.local/bin" >> $HOME/.bashrc
echo "export LRS_LOG_LEVEL=None #Debug" >> $HOME/.bashrc
echo "export PYTHONWARNINGS='ignore:setup.py install is deprecated,ignore:Invalid dash-separated options,ignore:pkg_resources is deprecated as an API,ignore:Usage of dash-separated'" >> $HOME/.bashrc

echo "export _colcon_cd_root=${HOME}/ament_ws" >> $HOME/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc


echo "Creating repos and stretch_user directories..."
mkdir -p $HOME/.local/bin
mkdir -p $HOME/repos
mkdir -p $HOME/stretch_user
mkdir -p $HOME/stretch_user/log
mkdir -p $HOME/stretch_user/debug
mkdir -p $HOME/stretch_user/maps
mkdir -p $HOME/stretch_user/models
mkdir -p $HOME/stretch_user/$LOCALROBOT_NAME
touch $HOME/stretch_user/$LOCALROBOT_NAME/stretch_configuration_params.yaml
touch $HOME/stretch_user/$LOCALROBOT_NAME/stretch_user_params.yaml

echo "robot:
  model_name: SE4
  batch_name: calder
  " > $HOME/stretch_user/$LOCALROBOT_NAME/stretch_configuration_params.yaml


echo "Setting up user copy of robot factory data (if not already there) at $HOME/stretch_user/$HELLO_FLEET_ID"

sudo chown -R : $HOME/stretch_user
sudo chmod -R a-x,o-w,+X $HOME/stretch_user


export PATH=${PATH}:$HOME/.local/bin
export HELLO_FLEET_ID=$HELLO_FLEET_ID
export HELLO_FLEET_PATH=$HOME/stretch_user

echo "Ensuring correct version of params present..."
params_dir_path=$HELLO_FLEET_PATH/$HELLO_FLEET_ID
echo "Checking params directory path: $params_dir_path"

# Define file paths based on the provided directory
user_params="$params_dir_path/stretch_re1_user_params.yaml"
factory_params="$params_dir_path/stretch_re1_factory_params.yaml"
new_config_params="$params_dir_path/stretch_configuration_params.yaml"
new_user_params="$params_dir_path/stretch_user_params.yaml"


echo "###########################################"
echo "CREATING JAZZY AMENT WORKSPACE at ~/ament_ws"
echo "###########################################"

echo "Ensuring correct version of ROS is sourced..."
if [[ $ROS_DISTRO && ! $ROS_DISTRO = "jazzy" ]]; then
    echo "Cannot create workspace while a conflicting ROS version is sourced. Exiting."
    exit 1
fi
source /opt/ros/jazzy/setup.bash

if [[ -d ~/ament_ws ]]; then
    if [[ -z "$DOCKER_BUILD" ]]; then
        echo "You are about to delete and replace the existing ament workspace. If you have any personal data in the workspace, please create a back up before proceeding."
        prompt_yes_no(){
        read -p "Do you want to continue? Press (y/n for yes/no): " x
        if [ $x = "n" ]; then
                echo "Exiting the script."
                exit 1
        elif [ $x = "y" ]; then
                echo "Continuing to create a new ament workspace."
        else
            echo "Press 'y' for yes or 'n' for no."
            prompt_yes_no
        fi
        }
        prompt_yes_no
    else
        echo "The DOCKER_BUILD flag is enabled, creating a new ament workspace."
    fi
fi


export PATH=${PATH}:~/.local/bin
echo "Deleting ~/ament_ws if it already exists..."
sudo rm -rf ~/ament_ws
echo "Creating the workspace directory..."
mkdir -p ~/ament_ws/src

cd ~/ament_ws/
echo "Initializing rosdep..."
sudo rosdep init || true  # Don't fail if already initialized
echo "Updating rosdep indices..."
rosdep update --include-eol-distros

echo "Cloning the workspace's packages..."
cd ~/ament_ws/src

vcs import --input /tmp/stretch_ros2_jazzy.repos
echo "Fetch ROS packages' dependencies (this might take a while)..."
source /opt/ros/jazzy/setup.bash
cd ~/ament_ws/
echo "Updating apt-get package lists..."
sudo apt-get update -y


echo "Doing rosdep install"
rosdep install --rosdistro=jazzy -iy --from-paths src --skip-keys="hesai_ros_driver airy_lidar_filter_cpp"
echo "Doing apt-get remove"

# echo "Install web interface dependencies..."
# cd ~/ament_ws/src/stretch4_web_teleop
# pip3 install -r requirements.txt --ignore-installed --break-system-packages
# npm install --force
# npx playwright install
# echo "Generating web interface certs..."
# cd ~/ament_ws/src/stretch4_web_teleop/certificates
# curl -JLO "https://dl.filippo.io/mkcert/latest?for=linux/amd64"
# chmod +x mkcert-v*-linux-amd64
# sudo cp mkcert-v*-linux-amd64 /usr/local/bin/mkcert
# CAROOT=`pwd` mkcert --install
# mkdir -p ~/.local/share/mkcert
# rm -rf ~/.local/share/mkcert/root*
# cp root* ~/.local/share/mkcert
# mkcert ${HELLO_FLEET_ID} ${HELLO_FLEET_ID}.local ${HELLO_FLEET_ID}.dev localhost 127.0.0.1 0.0.0.0 ::1
# rm mkcert-v*-linux-amd64
# cd ~/ament_ws/src/stretch4_web_teleop
# touch .env
# echo certfile=${HELLO_FLEET_ID}+6.pem >> .env
# echo keyfile=${HELLO_FLEET_ID}+6-key.pem >> .env
# cd ~/ament_ws/


# Try to install python3-pcl from PPA, but don't fail if it's not available
# set +e
# sudo add-apt-repository ppa:sweptlaser/python3-pcl -y
# sudo apt-get update -y
# sudo apt-get install python3-pcl -y
# set -e

echo "Buid ROS 2 wokspace"
cd ~/ament_ws
colcon build
source ./install/setup.bash
