# ROS node for real-time egohands detection #

This is a ROS implementation of the CSAILVision Semantic Segmentation/Scene Parsing framework / Pyramid Scene Parsing Network (PSPNet) retrained on the egohands dataset. Despite being trained on human hands, the NN detects human skin in general.

#### Input ####

**RGB compressed image:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)

#### Output ####

**Compressed mask:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html) (0 indicates background, 1 for human skin)

## Getting Started ##

### Dependencies ###

Running the node requires a gpu. The model have been tested with Python 2.7 and 3.6.
 
#### Hardware ####

* RGBD Camera
* GPU >= 4000MiB

#### Python3 / pip3 ####
```
numpy
scipy
cv2
torch, torchvision
PIL
yacs
tqdm
```
#### Ros ####
```
rospy
actionlib
sensor_msgs
cv_bridge
ros_numpyros_numpyros_numpy
```

### Weights ###

Download the weights from [Google Drive](https://drive.google.com/drive/u/1/folders/1q--u3g9XgQ0qH1I6JJfCs3EfTMc3t1IT) and place them in [/src/helper_CSAILVision/lib/segmentation](/src/helper_CSAILVision/lib/segmentation/).

### Bilding ###

*Optional:* To maximize performance, use the 'release' build mode:
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Configuration

The initial setup can be changed by adapting the [egohands.yaml](cfg/egohands.yaml) file:

#### Camera ####
* **topic:** Rostopic the publisher node is subcribing to.

#### Interfaces ####
* **topic:** Rostopic the publisher node is publishing to *(please do not change)*.
* **service:** Rosservice for interacting with the service node *(please do not change)*.
* **action:** Rostopic for interacting with the action node *(please do not change)*.

#### Visualization ####

The visualization mode published the original image with the background blacked out. Please be aware that turing on the visualization increases computing time and network utilization substantially.

* **topic:** Topic the node is publishing to.
* **activated:** Turn on/off visualization: *use keywords "on" or "off"*.

### Launch

The ros package contains 3 launch files:
* **Publisher:** The [publisher](launch/egohands_publisher.launch) launch file starts a ros node that published a new mask every time a new rgb image is published.
* **Serivce:** The [serivce](launch/egohands_service.launch) launch file starts a ros service. 
* **Action:** The [action](launch/egohands_action.launch) launch file starts a ros action server.

## Acknowledgments

The ROS node is powered by the Pyramid Scene Parsing Network (PSPNet) of [CSAILVision](https://github.com/CSAILVision/semantic-segmentation-pytorch).

The egohands dataset training has been done as part of a [student's project](https://github.com/junwenkwan/hand-seg-tpv) at [University Monash](https://www.monash.edu/)

## License

* **Academic:** This project is licensed under the 4-clause BSD License.
* **Commercial:** Please contact the author.
