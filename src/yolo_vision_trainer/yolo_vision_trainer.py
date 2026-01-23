'''
Docstring for urc-2023.src.yolo_vision_trainer.yolo_vision_trainer
This python file is responsible for training the yolov8l-world model to be more fine tuned to detect the mission objects (rock hammer, hammer and bottle)
It is not a node, and should not be running during rover operation
'''
from ultralytics import YOLO
import torch

def pre_train_model():
    # Check if GPU is available
    print("CUDA Available:", torch.cuda.is_available())

    # Get GPU details
    if torch.cuda.is_available():
        print("GPU Name:", torch.cuda.get_device_name(0))

    # Initialize a YOLO-World model
    model = YOLO("yolov8l-world.pt")  # or select yolov8m/l-world.pt
    model.info()
    
    #yolo model can be optimized to search for specific items with the line below
    model.set_classes(["hammer", "bottle"])

    # Save the model with the defined offline vocabulary
    model.save("trained_yolov8l.pt")

if __name__ is "__main__":
    pre_train_model()
