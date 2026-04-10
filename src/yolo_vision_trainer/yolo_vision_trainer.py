'''
Docstring for urc-2023.src.yolo_vision_trainer.yolo_vision_trainer
This python file is responsible for training the yolov8l-world model to be more fine tuned to detect the mission objects (rock hammer, hammer and bottle)
It is not a node, and should not be running during rover operation

#### FOR GPU SUPPORT ####
if the code below says cuda is unavailable, check that we have the correct torch version
running nvidia-smi returns
NVIDIA-SMI 540.4.0 Driver Version: 540.4.0 CUDA Version: 12.6
gpu 0, name : Orin (nvgpu)

pytorch says to use this version for cuda support on linux:
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu126

since we have it installed the command is 
pip3 install --upgrade torch torchvision --index-url https://download.pytorch.org/whl/cu126

last cuda version: (2.10.0+cu126)

current state: we get the following error on line 62
torch.AcceleratorError: CUDA error: no kernel image is available for execution on the device
Search for `cudaErrorNoKernelImageForDevice' in https://docs.nvidia.com/cuda/cuda-runtime-api/group__CUDART__TYPES.html for more information.
'''
import os

import torch
from ultralytics import YOLO


def pre_train_model(gpu_name = ""):
    parent_dir = os.path.dirname(os.getcwd())
    print("Parent Path: " + parent_dir)
    # Initialize a YOLO-World model

    #check if local model is installed
    file_path = parent_dir+"/"+"yolov8l-world.pt"
    assert os.path.isfile(file_path), f"File {file_path} does not exist"
    #note: running YOLO("yolov8l-world.pt") will download the model from their github

    #load local model
    model = YOLO(file_path)  # or select yolov8m/l-world.pt
    #load model onto gpu
    if(gpu_name != ""):
        model.to(gpu_name)
    print("Printing model info:")
    model.info()
    
    #yolo model can be optimized to search for specific items with the line below
    model.set_classes(["hammer", "bottle"])

    # Save the model with the defined offline vocabulary
    model.save(parent_dir+"/"+"trained_yolov8l.pt")

def gpu_check():
    #code to do cuda and fallback on cpu
    #device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    #tensor = tensor.to(device)   

    print("Starting cuda check")
    # Check if GPU is available
    print("Torch Version:",torch.__version__)
    print("Torch Version Cuda:",torch.version.cuda)
    print("CUDA Available:", torch.cuda.is_available())
    

    # Get GPU details
    if torch.cuda.is_available():
        print("GPU Name:", torch.cuda.get_device_name(0))
        return "cuda"
    else:
        print("GPU Unavailable")
        return ""


if __name__ == "__main__":
    print("gpu check")
    gpu = gpu_check()
    print("launching pre_training")
    #pre_train_model(gpu)
    
    '''
    D:\DevProj\urc-2023\home_comp_enviro\Scripts\Activate.ps1
    git clone https://github.com/heartexlabs/label-studio-converter.git
    cd label-studio-converter
    pip install -e . 

    label-studio-converter import coco -h  # just print help

    label-studio-converter import coco -i your-input-file.json -o output.json
    label-studio-converter import coco -i "D:\DevProj\urc-2023\src\yolo_vision_trainer\My First Project.coco\train\_annotations.coco.json" -o "D:\DevProj\urc-2023\src\yolo_vision_trainer\label-studio-project\output.json"
    

    python3.9 -m venv new_home_comp_enviro
    
    '''