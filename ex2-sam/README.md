# Example 2: sam

First, install the SAM checkpoint model. In the `ex2-sam` directory,
```
curl https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.p
th --output ./models/sam_vit_h_4b8939.pth
```

```
ros2 run sam sam_client
ros2 run sam sam_service
``