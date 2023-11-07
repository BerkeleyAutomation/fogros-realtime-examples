# Example 2: sam

First, install [Segment Anything](https://github.com/facebookresearch/segment-anything) (SAM) and the SAM checkpoint model.
```
pip install git+https://github.com/facebookresearch/segment-anything.git
curl https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth --output ./models/sam_vit_h_4b8939.pth
```

If you have an NVIDIA gpu: Install torch with CUDA enabled
```
pip3 install torch==1.13.1+cu117 torchvision>=0.13.1+cu117 --extra-index-url https://download.pytorch.org/whl/cu117 --no-cache-dir 
```
If you do not have an NVIDIA gpu, skip installing torch with CUDA and comment this line in `yolo_service.py`:
```py
self.sam.to(device="cuda")
```

Run the ros nodes
```
ros2 run sam sam_client
ros2 run sam sam_service
``