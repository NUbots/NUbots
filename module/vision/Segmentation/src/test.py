import os

import matplotlib.pyplot as plt
import numpy as np
import torch
import torchvision.transforms as transforms
from PIL import Image

# Import the model definition
from train import EfficientSegmentationModel


def visualize_segmentation():
    # Parameters
    model_path = "results/best_segmentation_model.pth"  # Updated path
    test_image_path = "test/images/81-test_nagoya_game_a_00278.png"  # Change this to your test image
    # Try to find corresponding ground truth mask
    test_mask_path = test_image_path.replace("images", "segmentations")
    img_size = 512

    # Define the class colors for visualization
    class_colors = [
        [254, 254, 254],  # Field - #FEFEFE
        [127, 127, 127],  # Field lines - #7F7F7F
        [0, 0, 0],  # Background - #000000
    ]

    # Create results directory if it doesn't exist
    os.makedirs("results", exist_ok=True)

    # Load the model
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # Updated to use the new model
    model = EfficientSegmentationModel(in_channels=3, num_classes=3)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.to(device)
    model.eval()

    # Prepare image transformations
    transform = transforms.Compose(
        [
            transforms.Resize((img_size, img_size)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    # Load and preprocess the test image
    if not os.path.exists(test_image_path):
        print(f"Error: Test image not found at {test_image_path}")
        return

    image = Image.open(test_image_path).convert("RGB")
    original_size = image.size

    # Save original image for display
    original_image = np.array(image)

    # Load ground truth mask if it exists
    ground_truth_exists = os.path.exists(test_mask_path)
    if ground_truth_exists:
        mask = Image.open(test_mask_path).convert("RGB")
        mask = mask.resize(original_size, Image.NEAREST)  # Ensure it's the same size
        ground_truth_mask = np.array(mask)
    else:
        ground_truth_mask = np.zeros_like(original_image)
        print(f"Warning: Ground truth mask not found at {test_mask_path}")

    # Transform image for model input
    image_tensor = transform(image).unsqueeze(0).to(device)

    # Run inference
    with torch.no_grad():
        output = model(image_tensor)

    # Process the output
    output = output.squeeze(0).cpu()
    _, predicted = torch.max(output, 0)
    predicted = predicted.numpy()

    # Create colored segmentation map
    segmentation_map = np.zeros((img_size, img_size, 3), dtype=np.uint8)
    for class_idx, color in enumerate(class_colors):
        segmentation_map[predicted == class_idx] = color

    # Resize segmentation map back to original image size if needed
    segmentation_image = Image.fromarray(segmentation_map)
    segmentation_image = segmentation_image.resize(original_size, Image.NEAREST)
    segmentation_map = np.array(segmentation_image)

    # Display the results
    plt.figure(figsize=(18, 6))

    plt.subplot(1, 3, 1)
    plt.title("Original Image")
    plt.imshow(original_image)
    plt.axis("off")

    plt.subplot(1, 3, 2)
    plt.title("Segmentation Result")
    plt.imshow(segmentation_map)
    plt.axis("off")

    plt.subplot(1, 3, 3)
    plt.title("Ground Truth Mask" if ground_truth_exists else "Ground Truth Not Found")
    plt.imshow(ground_truth_mask)
    plt.axis("off")

    plt.tight_layout()
    plt.savefig("results/segmentation_result.png")
    plt.show()

    print("Visualization saved as 'results/segmentation_result.png'")


if __name__ == "__main__":
    visualize_segmentation()
