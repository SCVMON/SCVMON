# SCVMON
The repository contains SCVMON's attack detection and recovery code.

## Setup
We tested SCVMON on ubuntu18.04.
The environment setup below is done targeting ubuntu18.04.

### Installation 

#### Clone SCVMON:
```
cd ~
git clone https://github.com/SCVMON/SCVMON.git
```

#### Install dependencies
```
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
```

#### Install mavproxy
```
sudo pip install future pymavlink MAVProxy
```

#### Install QgroundControl
For compatibility reasons, we use an older version of QgroundControl.
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager
```
Download older version of QgroundControl : https://github.com/mavlink/qgroundcontrol/releases/tag/v4.0.11

```
chmod +x ./QGroundControl.AppImage 
./QGroundControl.AppImage  (or double click)
```


#### Install Gazebo
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo9 libgazebo9-dev
```

#### Install Gazebo plugin for APM
```
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

### Build and simulate SCVMON 

Terminal 1: Gazebo 
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

Terminal 2: SITL
```
cd ~/SCVMON/Tools/autotest
./sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

## Attack Simulation

In order to simulate the current attack, the source code needs to be modified.
We modify the code at the location of the variable where the RV control program input is stored to simulate an attack that changes the input.
(The location of the corresponding attack module can be found through the t_a1 variable.)

In the near future, we plan to add code that simulates an attack by receiving input in the form of a script.
In this case, there is no need to re-build SCVMON each time another attack is simulated.


