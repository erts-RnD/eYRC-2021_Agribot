<div align="center">
  <h1>
    ebot_hardware
  </h1>
  <h4>Packages used in embedded device for ebot hardware</h4>
</div>

[![License: AGPL v3](https://img.shields.io/badge/License-AGPL_v3-blue.svg)](https://www.gnu.org/licenses/agpl-3.0)

## Usage

* Create new workspace using this command, 
    ```
    mkdir -p ~/agribot_hardware_ws/src
    cd ~/agribot_hardware_ws/src/
    git clone -b agribot_hardware https://github.com/erts-RnD/eYRC-2021_Agribot.git
    cd ~/agribot_hardware_ws/
    catkin_make
    ```

- Connecting to remote network.

Inside the ebot_hardware folder you will see a `connect.bash` file, Go to the file location and run  the following commands"

```shell
chmod u+x connect.bash
./connect.bash
```

After running the command for first time it will point you the dependencies required. Install those dependencies.

This will create a file inside ~/.config/raf by name credentials.yaml.

Edit the username and password assigned to you in this file and save it.

Now again run connect.bash file. You will now be connected to e-Yantra network.

Test the network by doing the following test:

```shell
ping 192.168.255.14
```

If ping is successful, your connection is working properly. 

Change the ROS Master and Host IP as mentioned in mdbook inside ~/.bashrc file. 

```bash
export ROS_MASTER_URI=http://192.168.255.14:11311 
export ROS_IP=<IP obtained from connect script>
```

Now you will be able to connect remote ROS.

> For using ROS locally in your system, disconnect the remote network by killing the connect.bash file comment the two lines added in .bashrc and run ROS on a new terminal.

More details about ROS multisystem is given in the mdbook.

## License

The code in this repository has been released under the [GNU Affero General Public License v3](https://www.gnu.org/licenses/agpl-3.0.en.html)

## Requirement

* **ur-msgs**: `sudo apt install ros-\<ROS-DISTRO\>-ur-msgs`

***Note that you have to make changes in your bashrc to make this repo work***
