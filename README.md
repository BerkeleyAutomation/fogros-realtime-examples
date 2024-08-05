# fogros-realtime-examples


```bash
colcon build
rosdep install --from-paths . --ignore-src -r -y
source install/setup.bash
ros2 run <package> <name>
```

If you see a warning during `colcon build`, you may safely ignore it:
```
/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
```


```
docker build . -t keplerc/fogros-ft-examples:latest
docker push  keplerc/fogros-ft-examples:latest
```