## Easy IMU Ros2
This the **ROS2** Package for the using the **`Easy IMU Module`** (**`MPU9250 EIMU Module`**) with **ROS2** in a PC or microcomputer, after successful setup with the [eimu_setup_application](https://github.com/samuko-things-company/eimu_setup_application).

> [!NOTE]  
> It should be used with your ros2 project running on linux `Ubuntu 22.04` [`ros-humble`] (e.g Raspberry Pi, PC, etc.)

## How to Use the Package
- ensure you've already set up your microcomputer or PC system with [`ros-humble`](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) with [`colcon`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) and your `ros workspace` also setup

- install the `libserial-dev` package on your linux machine
  ```shell
  sudo apt-get update
  sudo apt install libserial-dev
  ```

- install `rosdep` so you can install necessary ros related dependencies for the package.
  ```shell
  sudo apt-get update
  sudo apt install python3-rosdep2
  sudo rosdep init
  rosdep update
  ```

- cd into the **`src/`** folder of your **`ros workspace`** and clone the repo
  (or you can download and add it manually to the `src/` folder)
  > *NOTE: if you download it, extract it and change the folder name to `eimu_ros2` before moving it to the `src/` folder*
  ```shell
  git clone https://github.com/samuko-things-company/eimu_ros2.git
  ```

- from the **`src/`** folder, cd into the root directory of your **`ros workspace`** and run rosdep to install all necessary ros dependencies
  ```shell
  cd ../
  rosdep install --from-paths src --ignore-src -r -y
  ```

- connect the **`Easy IMU Module`** to the Computer (PC or microcomputer) and check its serial port:
  > The best way to select the right serial port (if you are using multiple serial devices) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > You should see a value printed on the terminal (if the driver is connected and seen by the computer), your serial port would be -> **/dev/serial/by-path/[value]**. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  ```shell
  ls /dev/ttyU*
  ```
  > you should see **/dev/ttyUSB0** or **/dev/ttyUSB1** and so on

- go to the `config` folder inside the **`eimu_ros2`** package folder. You'll see two params file. Change the serial port value to that found in the previous step. You don't need to change the `frame_id` and `publish_frequncy` values. leave the `publish_tf_on_map_frame` value as it is in both param files.

- build the packages with colcon (in your **`ros workspace`** root folder):
  ```shell
  colcon build --packages-select eimu_ros2 --symlink-install
  ```

- to vizualize in rviz (i.e quick test to see the IMU working), run:
  > *don't forget to source your `ros workspace`*
  ```shell
  ros2 launch eimu_ros2 test_eimu_ros2.launch.py
  ``` 
  in another terminal run: 
  ```shell
  rviz2
  ```
  > Add TF and move the IMU about to see the transform from the imu frame to the map frame for test.

- to use in your project (e.g with a URDF file).
  > Ensure the name of the imu link frame in your URDF FILE is the same as that of the `frame_id` in the `eimu_ros2_start_params.yaml`
  
  First launch or run your robot's package file, then run:
  > *don't forget to source your `ros workspace`*
  ```shell
  ros2 launch eimu_ros2 start_eimu_ros2.launch.py
  ```
  > the imu data should now be published with (or on) the robot's imu link frame.

>*NOTE: Feel free to use/edit the package as you see fit on your project.*

  
