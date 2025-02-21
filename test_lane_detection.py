import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt
from utils.lane_detector import LaneDetector
from models.enet import ENet
import os

# Define dataset and checkpoint paths
# DATASET_PATH = "X:/archive/TUSimple/test_set"
DATASET_PATH = "/opt/data/TUSimple/test_set"
CHECKPOINT_PATH = "checkpoints/enet_checkpoint_epoch_best.pth"  # Path to the trained model checkpoint

# Function to load the ENet model
def load_enet_model(checkpoint_path, device="cuda"):
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
    # print(f"Before transformation: image min/max = {image.min()}/{image.max()}")
    ####################### TODO: Your code starts Here #######################
    height, width = image.shape[:2]
    scaley = 256/720
    scalex = 512/1280
    # Define source points
    src_pts = np.float32([
        [scaley*2,scalex*717],   # Bottom-left
        [scaley*1276,scalex*719],   # Bottom-right
        [scaley*482,scalex*335],   # Top-left
        [scaley*757,scalex*331]    # Top-right
    ])

    # Define destination points for the Bird's Eye View
    dst_pts = np.float32([
        [0, height],                  # Bottom-left
        [width, height],               # Bottom-right
        [0, 0],                        # Top-left
        [width, 0]                     # Top-right
    ])
    # print(f"Image shape: {image.shape[:2]}")
    # print(f"Source Points: {src_pts}")

    # print("Source Points:", src_pts)
    # print("Destination Points:", dst_pts)

    # transformation matrix
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    # print("Perspective Transform Matrix:\n", M)
    # perspective warp
    transformed_image = cv2.warpPerspective(image, M, (width, height))
    ####################### TODO: Your code ends Here #######################
    # print(f"After transformation: transformed min/max = {transformed_image.min()}/{transformed_image.max()}")

    # plt.imshow(transformed_image, cmap="gray")
    # plt.title("Perspective Transform Output")
    # plt.show()
    
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
    fig, axes = plt.subplots(2, num_images, figsize=(15, 5))
    
    ####################### TODO: Your code starts Here #######################

    for i in range(num_images):
        # Resize the original image to 512 × 256
        image = cv2.resize(images[i], (512, 256))

        # Resize instance_map to match image dimensions
        instance_map = cv2.resize(instances_maps[i], (image.shape[1], image.shape[0]))

        # Normalize instance_map values if necessary
        if instance_map.max() > 3:  
            instance_map = instance_map.astype(np.uint8)
        else:
            instance_map = (instance_map / instance_map.max()) * 255
            instance_map = instance_map.astype(np.uint8)

        # Convert grayscale instance map to 3-channel for proper blending
        if len(instance_map.shape) == 2:
            instance_map = cv2.cvtColor(instance_map, cv2.COLOR_GRAY2BGR)

        # Apply perspective transformation to both the image and the instance map
        bird_eye_view = perspective_transform(image)
        bird_eye_instance_map = perspective_transform(instance_map)

        # Convert images to float32 for overlaying
        image = image.astype(np.float32)
        bird_eye_view = bird_eye_view.astype(np.float32)
        instance_map = instance_map.astype(np.float32)
        bird_eye_instance_map = bird_eye_instance_map.astype(np.float32)

        # Overlay instance maps onto both the regular and transformed images
        overlay_original = cv2.addWeighted(image, 1 - alpha, instance_map, alpha, 0).astype(np.uint8)
        overlay_transformed = cv2.addWeighted(bird_eye_view, 1 - alpha, bird_eye_instance_map, alpha, 0).astype(np.uint8)

        # Apply colormap for better visualization
        overlay_original_colormap = cv2.applyColorMap(overlay_original, cv2.COLORMAP_VIRIDIS)
        overlay_transformed_colormap = cv2.applyColorMap(overlay_transformed, cv2.COLORMAP_VIRIDIS)

        # Display original lane detection overlay (left half of the row)
        axes[0,i].imshow(cv2.cvtColor(overlay_original_colormap, cv2.COLOR_BGR2RGB))
        axes[0,i].set_title(f"Image {i+1} - Regular View")
        axes[0,i].axis("off")

        # Display transformed lane detection overlay (right half of the row)
        axes[1,i].imshow(cv2.cvtColor(overlay_transformed_colormap, cv2.COLOR_BGR2RGB))
        axes[1,i].set_title(f"Image {i+1} - Bird’s Eye View")
        axes[1,i].axis("off")
        
    ####################### TODO: Your code ends Here #######################

    plt.tight_layout()
    plt.show()

def main():
    # Initialize device and model
    device = "cuda" if torch.cuda.is_available() else "cpu"
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

