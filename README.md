# ROS Application Template for Communication with 'AR Glass ROS Driver'

This package is provided as an example template on how to communicate with the AR Glass ROS Driver.

The example provided in this application performs the following
1. Requests an image from AR Glass ROS Driver
    > Request will be done using a service call to the AR Glass ROS Driver. The AR Glass ROS Driver will reply to the service call with an image.
2. Process the Received Image
    > Here user can add his own functionality. (Ex: Draw object detection markers on the received image). The output should be bounding box coordinates
3. Send bounding box information to the AR Glass ROS Driver
   > AR Glass ROS Driver is subscribed to a bounding box information topic. Any message published to this topic will be sent to the AR Glass. The application will publish the bounding box coordinates to this topic.


## Dependencies
* ROS
* Python 2.7x
* ar_glass (AR Glass ROS Driver package)

Tested on Ubuntu 16.04 ROS Kinetic

**Compiling** <br /> 
* Copy the package into catkin workspace. 
    > (For information regading ROS installation and creating a catkin workspace, please refer ROS installation tutorials)
* Compile the package
    ```
    # Open a terminal and go to catkin workspace directory
    cd ~/catkin_ws

    # Compile the package
    catkin_make --only-pkg-with-deps user_application
    ```
    > **Note**: ar_glass ROS Package (ar_glass-driver) must be present in the workspace folder, in order to compile this pacakge.

* Run the package
    ```
    roslaunch user_application application.launch
    ```
    > **Note**: ar_glass package should be launched before launching this package. 

#### Published Topics
- ar_glass/BoundingBox : /AR_Send_Image
  
    ```             
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    int32 left_x
    int32 right_x
    int32 top_y
    int32 bottom_y
    ```
            
  
#### Services
- ar_glass/Image: /AR_Take_Image


#### Permissions:
In some cases, it might be required to provide execution permissions to the python executable.
>       sudo chmod +x user_application/scripts/application.py

#### C++ Portability
This package is written using Python 2.7. If required this can also be supplied using C++. Please contact us if you need the C++ package. 

