#chmod +x setup_vision_venv.sh
#never run this script with sudo, since source breaks

python3 -m venv vision_packages --system-site-packages
source vision_packages/bin/activate
which python

#NumPy compatible with cv_bridge
pip install "numpy<2"

# install JetPack PyTorch
pip install --no-cache \
  https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl

# install JetPack torchvision
pip install --no-cache torchvision \
  --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126

# install YOLO
pip install ultralytics==8.2.0


touch vision_packages\COLCON_IGNORE
