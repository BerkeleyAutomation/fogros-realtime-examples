# first build an image with rust, ros and cargo chef
FROM ros:humble AS chef

RUN apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp

# YOLO 
RUN apt-get update && apt-get install -y python3-pip 
RUN pip3 install numpy ultralytics
# SAM 
RUN apt-get install -y wget 
RUN pip3 install git+https://github.com/facebookresearch/segment-anything.git 
RUN wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
RUN mv sam_vit_h_4b8939.pth /tmp/sam_vit_h_4b8939.pth

WORKDIR /ros_ws 
RUN mkdir -p /ros_ws/src/fogros-realtime-examples
COPY . ./src/fogros-realtime-examples
RUN . /opt/ros/humble/setup.sh && colcon build 

# && colcon build --packages-select apriltag_msgs && colcon build