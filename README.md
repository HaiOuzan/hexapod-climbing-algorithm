#### 1. Realsense camera installation: ####

need first to install realsense-viwer

sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -y --no-install-recommends \
    python3 \
    python3-setuptools \
    python3-pip \
	python3-dev

# Install the core packages required to build librealsense libs
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
# Install Distribution-specific packages for Ubuntu 18
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Install LibRealSense from source
# We need to build from source because
# the PyPi pip packages are not compatible with Arm processors.
# See link [here](https://github.com/IntelRealSense/librealsense/issues/6964).

# also need to install realsense-viwer

# First clone the repository
git clone https://github.com/IntelRealSense/librealsense.git
cd ./librealsense

# Make sure that your RealSense cameras are disconnected at this point
# Run the Intel Realsense permissions script
./scripts/setup_udev_rules.sh

# Now the build
mkdir build && cd build
## Install CMake with Python bindings (that's what the -DBUILD flag is for)
## see link: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python#building-from-source
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true
## Recompile and install librealsense binaries
## This is gonna take a while! The -j4 flag means to use 4 cores in parallel
## but you can remove it and simply run `sudo make` instead, which will take longer
sudo make uninstall && sudo make clean && sudo make -j4 && sudo make install

## Export pyrealsense2 to your PYTHONPATH so `import pyrealsense2` works
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2


#### 2. installing code-oss ####
## https://www.youtube.com/watch?v=Fegmuh6_mEg&t=600s

sudo apt-get update
cd cd Downloads/
sudo apt-get insatll curl
curl -L https://github.com/toolboc/vscode/releases/download/1.32.3/code-oss_1.32.3-arm64.deb -o code-oss_1.32.3-arm64.deb
sudo dpkg -i code oss_1.1.32.3-arm64.deb
code-oss



#### 3. installing DYNAMIXEL ####
sudo apt install git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
sudi apt install python3
cd DynamixelSDK/python/
sudo python setup.py install


#### 4. upgrade numpy to version 1.19.3 ####

# First we have to delete numpy fron the jetson

pip3 uninstall numpy

# Then makre sure cthon is install

pip3 install cython

# Install numpy

pip3 install numpy
# Its can take a few minutes 

# check if it work

python3
import numpy
numpy.__version__


#### 5. install pygame ####

1. sudo apt-get install python3-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev libsdl2-dev libsmpeg-dev python3-numpy subversion libportmidi-dev ffmpeg libswscale-dev libavformat-dev libavcodec-dev libfreetype6-dev

# use this version only
2. python3 -m pip install -U pygame==1.9.6


#### 6. fix the connection issue with the xbox controller ####

open terminal and run:
sudo nano /etc/modprobe.d/xbox_bt.conf

# Add this line and save the file
options bluetooth disable_ertm=1
(ctr+o >> enter >> ctr+x)

sudo reboot

#### 7. If the usb makes problem run this command on terminal ####

sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/ttyUSB1