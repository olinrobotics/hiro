# Robot Arm Plays Set

For this project, we programmed [a robotic arm Edwin](https://github.com/olinrobotics/irl) to play [Set](https://en.wikipedia.org/wiki/Set_(game))

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

Install `usb_camera` [here](https://github.com/olinrobotics/usb_cam).

### Running the code

Set up ROS Environment


```
roscore
```
Connect to Edwin node

```
rosrun irl edwin_node.py
```

Set up edwin routes and behaviors
```
rosrun irl edwin_routes.py
rosrun irl edwin_behaviors.py
```

Connect to camera
```
rosrun usb_cam usb_cam_node _video_device:='/dev/video1' _image_width:=1280 _image_height:=720
```

Main class of Set game
```
python set_main.py
```

## Built With
* ROS
* OpenCV


## Authors

* [**Cassandra Overney**](https://github.com/coverney)
* [**Enmo Ren**](https://github.com/Enmoren)
* [**Khang Vu**](https://github.com/minhkhang1795)
* [**Emma Pan**](https://github.com/epan547)


## Acknowledgments

* [Olin Robotics](https://olinrobotics.github.io)
