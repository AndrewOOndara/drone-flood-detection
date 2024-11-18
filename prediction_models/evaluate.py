import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from sklearn.model_selection import train_test_split
import os
from PIL import Image
import numpy as np
from tqdm import tqdm

class FloodCNN(nn.Module):
    def __init__(self):
        super(FloodCNN, self).__init__()
        
        self.features = nn.Sequential(
            # First conv block
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(2),
            
            # Second conv block
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(2),
            
            # Third conv block
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2),
            
            # Fourth conv block
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.BatchNorm2d(256),
            nn.MaxPool2d(2)
        )
        
        self.classifier = nn.Sequential(
            nn.Dropout(0.5),
            nn.Linear(256 * 8 * 8, 512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512, 1),
            nn.Sigmoid()
        )

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), -1)
        x = self.classifier(x)
        return x

class FloodEvaluator:
    def __init__(self, model_path: str, device: torch.device = None):
        """
        Initialize the evaluator.

        Args:
            model_path (str): Path to the saved model weights (.pth file)
            device (torch.device): Device to run the model on. If None, will use CUDA if available
        """

        if device is None:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = device

        # Load and initialize the model
        self.model: FloodCNN = FloodCNN().to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

        # Define the same transforms as used in training (except for augmentation)
        self.transform = transforms.Compose([
            transforms.Resize((128, 128)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                            std=[0.229, 0.224, 0.225])
        ])

    def evaluate_one(self, image: Image.Image) -> tuple[int, float]:
        """
        Evalutate an image to detect flooding.

        Args:
            image_path (str): Path to the image file

        Returns:
            tuple: (prediction (0 or 1), confidence score (0-1))
        """
        # Preprocess the image
        image_tensor = (self.transform)(image).unsqueeze(0).to(self.device)  # Add batch dimension

        # Get prediction
        with torch.no_grad():
            output = (self.model)(image_tensor).squeeze()
            confidence = output.item()
            prediction = 1 if confidence >= 0.5 else 0

        return prediction, confidence


def evaluate_image(image_path, model_path='best_model.pth', device=None):
    """
    Evaluate a single image using a trained model.
    
    Args:
        image_path (str): Path to the image file
        model_path (str): Path to the saved model weights (.pth file)
        device (torch.device): Device to run the model on. If None, will use CUDA if available
    
    Returns:
        tuple: (prediction (0 or 1), confidence score (0-1))
    """
    if device is None:
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # Load and initialize the model
    model = FloodCNN().to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()
    
    # Define the same transforms as used in training (except for augmentation)
    transform = transforms.Compose([
        transforms.Resize((128, 128)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                           std=[0.229, 0.224, 0.225])
    ])
    
    # Load and preprocess the image
    try:
        image = Image.open(image_path).convert('RGB')
        image_tensor = transform(image).unsqueeze(0).to(device)  # Add batch dimension
    except Exception as e:
        raise Exception(f"Error loading or processing image: {str(e)}")
    
    # Get prediction
    with torch.no_grad():
        output = model(image_tensor).squeeze()
        confidence = output.item()
        prediction = 1 if confidence >= 0.5 else 0
    
    return prediction, confidence

def batch_evaluate_images(image_dir, model_path='best_model.pth'):
    """
    Evaluate all images in a directory and its subdirectories.
    
    Args:
        image_dir (str): Directory containing images to evaluate
        model_path (str): Path to the saved model weights (.pth file)
    
    Returns:
        dict: Dictionary mapping image paths to (prediction, confidence) tuples
    """
    results = {}
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # Get all image files recursively
    image_files = get_image_files_recursive(image_dir)
    
    for image_path in tqdm(image_files, desc="Evaluating images"):
        try:
            prediction, confidence = evaluate_image(image_path, model_path, device)
            results[image_path] = (prediction, confidence)
        except Exception as e:
            print(f"Error processing {image_path}: {str(e)}")
            continue
    
    return results

# Example usage
if __name__ == '__main__':
    # Training part
        
    # Example of evaluating a single image
    try:
        image_path = "./path.png"
        prediction, confidence = evaluate_image(image_path)
        print(f"\nSingle image evaluation:")
        print(f"Image: {image_path}")
        print(f"Prediction: {'Flooded' if prediction == 1 else 'Not Flooded'}")
        print(f"Confidence: {confidence:.2%}")
    except Exception as e:
        print(f"Error evaluating image: {str(e)}")
    
    # # Example of batch evaluation
    # try:
    #     test_dir = "path/to/test/directory"
    #     results = batch_evaluate_images(test_dir)
        
    #     print("\nBatch evaluation results:")
    #     for image_path, (pred, conf) in results.items():
    #         print(f"Image: {image_path}")
    #         print(f"Prediction: {'Flooded' if pred == 1 else 'Not Flooded'}")
    #         print(f"Confidence: {conf:.2%}")
    #         print("-" * 50)
    # except Exception as e:
    #     print(f"Error in batch evaluation: {str(e)}")