<br />
<div align="center">
  <a href="https://github.com/atom-robotics-lab/assets/blob/main/logo_1.png?raw=true">
    <img src="https://github.com/atom-robotics-lab/assets/blob/main/logo_1.png?raw=true" alt="Logo" width="120" height="120">
  </a>
  <h3 align="center">Hexapod</h3>

  <p align="center">
    This is the repo for the <a href="https://github.com/atom-robotics-lab/Hexapod">Hexapod</a> Project, Our hexapod has six legs and can walk around on rough ground. We made it to work well outdoors where normal wheels might get stuck. Each leg moves on its own, so it can handle tricky spots better.
    <br />
  </p>
</div>

### About the Project

Our hexapod has six legs and can walk around on rough ground. We made it to work well outdoors where normal wheels might get stuck. Each leg moves on its own, so it can handle tricky spots better.

### Built With

* [![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)](https://www.sphinx-docs.org)
* [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
* [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)

## Getting Started

This is an example of how you may give instructions on setting up your project locally.

### Prerequisites
This is an example of how to list things you need to use the software and how to install them.

* Docker
  - Refer to this [docker installation guide](https://docs.docker.com/engine/install/)
### Installation
 
 1. Make a new workspace
    ```bash
    mkdir -p hexapod_ws/src
    ```

2. Clone the hexapod repository


    Now go ahead and clone this repository inside the "src" folder of the workspace you just created.

      ```bash
      cd hexapod_ws/src
      git clone git@github.com:atom-robotics-lab/Hexapod.git
 
      ```
3. Make Docker files executable
     ```bash
     chmod +x build.sh run.sh
     ```
   
4. Build the Docker
     ```bash
     sudo ./build.sh
     ```
5. Run the Docker
     ```bash
     sudo ./run.sh
     ```
![2 (1)](https://github.com/user-attachments/assets/d78174c2-c119-406a-a6fd-e5511e091c4f)
## Usage
Our package consists of two directories as follows:-

- The `hexapod_description` dir contains all the bot model description files.
- The `hexapod_control` dir contains config files for manuevering the robot.

### 1. Launch Gazebo
* Spawns our robot in a custom gazebo world along with all the necessary plugins.
    ```bash
    ros2 launch hexapod_description hexapod.launch.py
    ```
  
  

### 2. Launch Control file
* Launches files responsible for teleoperation of robot.
    ```bash
    ros2 run hexapod_control bot_controller.py
    ```
### 3. Run teleop_twist
* Node for directions and speed of the robot.
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
**_NOTE:_** Make sure to keep the speed of robot high.<br />
<p align="center">
  <a href="https://www.youtube.com/watch?v=xpnilti2U3U">
    <img src="https://img.youtube.com/vi/xpnilti2U3U/0.jpg" alt="Watch the video" width="600">
  </a>
</p>

## Roadmap

- [x] Alpha version
- [x] Version 1
    - [x] Custom Inverse Kinematics
    - [x] Dockerize setup
    - [ ] Hardware prototype

## Contributing

We wholeheartedly welcome contributions!  
They are the driving force that makes the open-source community an extraordinary space for learning, inspiration, and creativity. Your contributions, no matter how big or small, are **genuinely valued** and **highly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/New-Feature`)
3. Commit your Changes (`git commit -m 'Add some New-Feature'`)
4. Push to the Branch (`git push origin feature/New-Feature`)
5. Open a Pull Request

Please adhere to this project's `code of conduct`.

## License

[APACHE 2.0](https://choosealicense.com/licenses/apache-2.0/)

## Contact Us

If you have any feedback, please reach out to us at:  
Our Socials - [Linktree](https://linktr.ee/atomlabs)

## Acknowledgments

* [Our wiki](https://atom-robotics-lab.github.io/wiki)
* [ROS Official Documentation](http://wiki.ros.org/Documentation)
* [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
* [Ubuntu Installation guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
* [Docker installation guide](https://docs.docker.com/engine/install/ubuntu/)






  

  
   
