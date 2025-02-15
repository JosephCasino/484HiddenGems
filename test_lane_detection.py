import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt
from utils.lane_detector import LaneDetector
from models.enet import ENet
import os

# Define dataset and checkpoint paths
DATASET_PATH = "/Users/yuan/Library/CloudStorage/OneDrive-UniversityofIllinois-Urbana/Courses/25SPRING/ECE484/MP1/archive/TUSimple/test_set"
CHECKPOINT_PATH = "checkpoints/enet_checkpoint_epoch_best.pth"  # Path to the trained model checkpoint

# Function to load the ENet model
def load_enet_model(checkpoint_path, device="cpu"):
    enet_model = ENet(binary_seg=2, embedding_dim=4).to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    enet_model.load_state_dict(checkpoint['model_state_dict'])
    enet_model.eval()
    return enet_model

def perspective_transform(image):
    """
    Transform an image into a bird's eye view.
        1. Calculate the image height and width.
        2. Define source points on the original image and corresponding destination points.
        3. Compute the perspective transform matrix using cv2.getPerspectiveTransform.
        4. Warp the original image using cv2.warpPerspective to get the transformed output.
    """
    
    ####################### TODO: Your code starts Here #######################
    height, width = image.shape[:2]

    # Define source points
    src_pts = np.float32([
        [1123,514],   # Bottom-left
        [961,514],   # Bottom-right
        [1123,673],   # Top-left
        [961,673]    # Top-right
    ])

    # Define destination points for the Bird's Eye View
    dst_pts = np.float32([
        [0, height],                  # Bottom-left
        [width, height],               # Bottom-right
        [0, 0],                        # Top-left
        [width, 0]                     # Top-right
    ])

    # transformation matrix
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # perspective warp
    transformed_image = cv2.warpPerspective(image, M, (width, height))
    ####################### TODO: Your code ends Here #######################
    
    return transformed_image


# Function to visualize lane predictions for multiple images in a single row
def visualize_lanes_row(images, instances_maps, alpha=0.7):
    """
    Visualize lane predictions for multiple images in a single row
    For each image:
        1. Resize it to 512 x 256 for consistent visualization.
        2. Apply perspective transform to both the original image and its instance map.
        3. Overlay the instance map to a plot with the corresponding original image using a specified alpha value.
    """
    
    num_images = len(images)
    fig, axes = plt.subplots(1, num_images, figsize=(15, 5))

    ####################### TODO: Your code starts Here #######################
    # Ensure axes is iterable (in case num_images = 1)
    if num_images == 1:
        axes = [axes]

    for i in range(num_images):
        # Resize the input image to 512x256 for consistency
        image = cv2.resize(images[i], (512, 256))

        # Apply the perspective transform
        bird_eye_view = perspective_transform(image)

        # Resize the instance map and overlay it on the transformed image
        instance_map = cv2.resize(instances_maps[i], (512, 256))  # Resize instance map
        overlay = cv2.addWeighted(bird_eye_view, 1 - alpha, instance_map, alpha, 0)

        # Display the final overlaid image
        axes[i].imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
        axes[i].axis("off")  # Hide axes for a cleaner display

    ####################### TODO: Your code ends Here #######################

    plt.tight_layout()
    plt.show()

def main():
    # Initialize device and model
    # device = "cuda" if torch.cuda.is_available() else "cpu"
    device = "cpu"
    enet_model = load_enet_model(CHECKPOINT_PATH, device)
    lane_predictor = LaneDetector(enet_model, device=device)

    # List of test image paths
    sub_paths = [
        "clips/0530/1492626047222176976_0/20.jpg",
        "clips/0530/1492626286076989589_0/20.jpg",
        "clips/0531/1492626674406553912/20.jpg",
        "clips/0601/1494452381594376146/20.jpg",
        "clips/0601/1494452431571697487/20.jpg"
    ]
    test_image_paths = [os.path.join(DATASET_PATH, sub_path) for sub_path in sub_paths]

    # Load and process images
    images = []
    instances_maps = []

    for path in test_image_paths:
        image = cv2.imread(path)
        if image is None:
            print(f"Error: Unable to load image at {path}")
            continue

        print(f"Processing image: {path}")
        instances_map = lane_predictor(image)
        images.append(image)
        instances_maps.append(instances_map)

    # Visualize all lane predictions in a single row
    if images and instances_maps:
        visualize_lanes_row(images, instances_maps)

if __name__ == "__main__":
    main()
