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
'''
from ultralytics import YOLO
import torch
import os

def pre_train_model():
    parent_dir = os.path.dirname(os.getcwd())
    print("Parent Path: " + parent_dir)

    print("Starting cuda check")
    # Check if GPU is available
    print("CUDA Available:", torch.cuda.is_available())

    # Get GPU details
    if torch.cuda.is_available():
        print("GPU Name:", torch.cuda.get_device_name(0))
    else:
        return

    # Initialize a YOLO-World model

    #check if local model is installed
    file_path = parent_dir+"/"+"yolov8l-world.pt"
    assert os.path.isfile(file_path), f"File {file_path} does not exist"
    #note: running YOLO("yolov8l-world.pt") will download the model from their github

    #load local model
    model = YOLO(file_path)  # or select yolov8m/l-world.pt
    print("Printing model info:")
    model.info()
    
    #yolo model can be optimized to search for specific items with the line below
    model.set_classes(["hammer", "bottle"])

    # Save the model with the defined offline vocabulary
    model.save(parent_dir+"/"+"trained_yolov8l.pt")

if __name__ == "__main__":
    print("launching pre_training")
    pre_train_model()
