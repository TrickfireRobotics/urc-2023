source vision_packages/bin/activate
python3 - << 'EOF'
import torch, cv2, numpy
print("Torch:", torch.__version__)
print("CUDA:", torch.cuda.is_available())
print("NumPy:", numpy.__version__)
print("OpenCV:", cv2.__version__)
from ultralytics import YOLO
EOF
