
docker run --net=host -ti keplerc/mpt:service ros2 run mpt_ros motion_plan_server

docker run --net=host -ti keplerc/mpt:service ros2 run mpt_ros motion_plan_client --ros-args -p scenario:=Twistycool -p experiment:=latency