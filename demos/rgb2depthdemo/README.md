# RGB to Depth

create conda env

```bash
conda create -n rgb2depth python=3.12
```

install dependencies

```bash
conda activate rgb2depth
pip install -r requirements.txt
```

install model

```bash
cd rgb2depthdemo
mkdir models
wget https://huggingface.co/depth-anything/Depth-Anything-V2-Large/resolve/main/depth_anything_v2_vitl.pth?download=true -O models/depth_anything_v2_vitl.pth
```

run demo

```bash
python rgb_to_depth.py
```
