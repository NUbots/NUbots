import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
import numpy as np
from tqdm import tqdm
import random
from torch.optim.lr_scheduler import ReduceLROnPlateau


# Define the SE (Squeeze-and-Excitation) block for channel attention
class SEBlock(nn.Module):
    def __init__(self, channel, reduction=4):
        super(SEBlock, self).__init__()
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.fc = nn.Sequential(
            nn.Linear(channel, channel // reduction, bias=False),
            nn.ReLU(inplace=True),
            nn.Linear(channel // reduction, channel, bias=False),
            nn.Sigmoid(),
        )

    def forward(self, x):
        b, c, _, _ = x.size()
        y = self.avg_pool(x).view(b, c)
        y = self.fc(y).view(b, c, 1, 1)
        return x * y.expand_as(x)


# Define a MobileNetV3-inspired block with depthwise separable convolutions
class MobileBlock(nn.Module):
    def __init__(self, in_channels, out_channels, stride=1, expansion=4, use_se=True):
        super(MobileBlock, self).__init__()
        self.use_residual = stride == 1 and in_channels == out_channels
        hidden_dim = in_channels * expansion

        self.conv = nn.Sequential(
            # Expansion
            nn.Conv2d(in_channels, hidden_dim, kernel_size=1, bias=False),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU6(inplace=True),
            # Depthwise
            nn.Conv2d(
                hidden_dim,
                hidden_dim,
                kernel_size=3,
                stride=stride,
                padding=1,
                groups=hidden_dim,
                bias=False,
            ),
            nn.BatchNorm2d(hidden_dim),
            nn.ReLU6(inplace=True),
        )

        # SE block
        self.se = SEBlock(hidden_dim) if use_se else nn.Identity()

        # Projection
        self.project = nn.Sequential(
            nn.Conv2d(hidden_dim, out_channels, kernel_size=1, bias=False),
            nn.BatchNorm2d(out_channels),
        )

    def forward(self, x):
        identity = x
        x = self.conv(x)
        x = self.se(x)
        x = self.project(x)
        if self.use_residual:
            x = x + identity
        return x


# Define efficient segmentation model with skip connections
class EfficientSegmentationModel(nn.Module):
    def __init__(self, in_channels=3, num_classes=3):
        super(EfficientSegmentationModel, self).__init__()

        # Initial conv layer
        self.init_conv = nn.Sequential(
            nn.Conv2d(in_channels, 16, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(16),
            nn.ReLU6(inplace=True),
        )

        # Encoder blocks
        self.enc1 = MobileBlock(16, 24, stride=2, use_se=True)
        self.enc2 = MobileBlock(24, 32, stride=2, use_se=True)
        self.enc3 = MobileBlock(32, 64, stride=2, use_se=True)

        # Decoder blocks with skip connections
        self.dec1 = nn.Sequential(
            nn.ConvTranspose2d(
                64, 32, kernel_size=3, stride=2, padding=1, output_padding=1
            ),
            nn.BatchNorm2d(32),
            nn.ReLU6(inplace=True),
        )

        self.dec2 = nn.Sequential(
            nn.ConvTranspose2d(
                64, 24, kernel_size=3, stride=2, padding=1, output_padding=1
            ),
            nn.BatchNorm2d(24),
            nn.ReLU6(inplace=True),
        )

        self.dec3 = nn.Sequential(
            nn.ConvTranspose2d(
                48, 16, kernel_size=3, stride=2, padding=1, output_padding=1
            ),
            nn.BatchNorm2d(16),
            nn.ReLU6(inplace=True),
        )

        # Final output layer
        self.final = nn.Conv2d(32, num_classes, kernel_size=1)

    def forward(self, x):
        # Encoder path with skip connections
        x0 = self.init_conv(x)  # [B, 16, H/2, W/2]
        x1 = self.enc1(x0)  # [B, 24, H/4, W/4]
        x2 = self.enc2(x1)  # [B, 32, H/8, W/8]
        x3 = self.enc3(x2)  # [B, 64, H/16, W/16]

        # Decoder path with skip connections
        d1 = self.dec1(x3)  # [B, 32, H/8, W/8]
        d1 = torch.cat([d1, x2], dim=1)  # [B, 64, H/8, W/8]

        d2 = self.dec2(d1)  # [B, 24, H/4, W/4]
        d2 = torch.cat([d2, x1], dim=1)  # [B, 48, H/4, W/4]

        d3 = self.dec3(d2)  # [B, 16, H/2, W/2]
        d3 = torch.cat([d3, x0], dim=1)  # [B, 32, H/2, W/2]

        out = self.final(d3)  # [B, num_classes, H/2, W/2]
        return nn.functional.interpolate(
            out, scale_factor=2, mode="bilinear", align_corners=False
        )


# Custom dataset for segmentation (with augmentation)
class SegmentationDataset(Dataset):
    def __init__(self, img_dir, mask_dir, transform=None, img_size=256, augment=False):
        self.img_dir = img_dir
        self.mask_dir = mask_dir
        self.transform = transform
        self.img_size = img_size
        self.augment = augment

        self.images = [f for f in os.listdir(img_dir) if f.endswith((".jpg", ".png"))]

    def __len__(self):
        return len(self.images)

    def apply_augmentation(self, image, mask):
        # Random horizontal flip
        if random.random() > 0.5:
            image = image.transpose(Image.FLIP_LEFT_RIGHT)
            mask = mask.transpose(Image.FLIP_LEFT_RIGHT)

        # Random rotation (up to 10 degrees)
        if random.random() > 0.5:
            angle = random.uniform(-10, 10)
            image = image.rotate(angle, Image.BILINEAR)
            mask = mask.rotate(angle, Image.NEAREST)

        # Random brightness/contrast adjustment (only to image)
        if random.random() > 0.5:
            from PIL import ImageEnhance

            factor = random.uniform(0.8, 1.2)
            image = ImageEnhance.Brightness(image).enhance(factor)
            factor = random.uniform(0.8, 1.2)
            image = ImageEnhance.Contrast(image).enhance(factor)

        return image, mask

    def __getitem__(self, idx):
        img_name = self.images[idx]
        img_path = os.path.join(self.img_dir, img_name)
        mask_path = os.path.join(self.mask_dir, img_name)

        # Handle potential extension differences
        if not os.path.exists(mask_path):
            base_name = os.path.splitext(img_name)[0]
            for ext in [".jpg", ".png"]:
                potential_mask_path = os.path.join(self.mask_dir, base_name + ext)
                if os.path.exists(potential_mask_path):
                    mask_path = potential_mask_path
                    break

        image = Image.open(img_path).convert("RGB")
        mask = Image.open(mask_path).convert("RGB")

        # Resize both image and mask to the same dimensions
        image = image.resize((self.img_size, self.img_size), Image.BILINEAR)
        mask = mask.resize((self.img_size, self.img_size), Image.NEAREST)

        # Apply augmentation if enabled
        if self.augment:
            image, mask = self.apply_augmentation(image, mask)

        if self.transform:
            image = self.transform(image)

            # Process the mask to convert colors to class indices
            mask_np = np.array(mask)

            # Create empty mask with class indices
            class_mask = np.zeros((mask_np.shape[0], mask_np.shape[1]), dtype=np.uint8)

            # Class mapping based on RGB values
            # Field - #FEFEFE (254, 254, 254)
            field_mask = np.all(mask_np == [254, 254, 254], axis=2)
            class_mask[field_mask] = 0

            # Field lines - #7F7F7F (127, 127, 127)
            lines_mask = np.all(mask_np == [127, 127, 127], axis=2)
            class_mask[lines_mask] = 1

            # Background - #000000 (0, 0, 0)
            bg_mask = np.all(mask_np == [0, 0, 0], axis=2)
            class_mask[bg_mask] = 2

            # Convert to tensor
            mask_tensor = torch.LongTensor(class_mask)

            return image, mask_tensor  # Return class index tensor directly

        return image, mask


# Improved training function with learning rate scheduling
def train_model(
    model, train_loader, val_loader, criterion, optimizer, device, num_epochs=20
):
    best_loss = float("inf")

    # Add learning rate scheduler for better convergence
    scheduler = ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=3, verbose=True
    )

    for epoch in range(num_epochs):
        # Training
        model.train()
        train_loss = 0.0

        for images, masks in tqdm(
            train_loader, desc=f"Epoch {epoch + 1}/{num_epochs} - Training"
        ):
            images = images.to(device)
            masks = masks.to(device)

            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, masks)
            loss.backward()
            optimizer.step()

            train_loss += loss.item() * images.size(0)

        train_loss = train_loss / len(train_loader.dataset)

        # Validation
        model.eval()
        val_loss = 0.0
        correct_pixels = 0
        total_pixels = 0

        with torch.no_grad():
            for images, masks in tqdm(
                val_loader, desc=f"Epoch {epoch + 1}/{num_epochs} - Validation"
            ):
                images = images.to(device)
                masks = masks.to(device)

                outputs = model(images)
                loss = criterion(outputs, masks)
                val_loss += loss.item() * images.size(0)

                # Calculate pixel accuracy
                preds = torch.argmax(outputs, dim=1)
                correct_pixels += (preds == masks).sum().item()
                total_pixels += masks.numel()

        val_loss = val_loss / len(val_loader.dataset)
        pixel_acc = 100 * correct_pixels / total_pixels

        # Update learning rate based on validation loss
        scheduler.step(val_loss)

        print(
            f"Epoch {epoch + 1}/{num_epochs}: "
            f"Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}, "
            f"Pixel Acc: {pixel_acc:.2f}%"
        )

        # Save best model
        if val_loss < best_loss:
            best_loss = val_loss
            torch.save(model.state_dict(), "results/best_segmentation_model.pth")
            print(f"Model saved with loss: {val_loss:.4f}")


def main():
    # Create results directory if it doesn't exist
    os.makedirs("results", exist_ok=True)

    # Set device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Set directories
    train_img_dir = "train/images"
    train_mask_dir = "train/segmentations"
    test_img_dir = "test/images"
    test_mask_dir = "test/segmentations"

    # Image transformations
    img_size = 512  # Use this size consistently
    transform = transforms.Compose(
        [
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    # Create datasets with consistent image size and augmentation for training
    train_dataset = SegmentationDataset(
        train_img_dir,
        train_mask_dir,
        transform=transform,
        img_size=img_size,
        augment=True,
    )
    test_dataset = SegmentationDataset(
        test_img_dir,
        test_mask_dir,
        transform=transform,
        img_size=img_size,
        augment=False,
    )

    # Create dataloaders
    batch_size = 8
    train_loader = DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True, num_workers=2
    )
    test_loader = DataLoader(
        test_dataset, batch_size=batch_size, shuffle=False, num_workers=2
    )

    # Initialize model
    model = EfficientSegmentationModel(in_channels=3, num_classes=3)
    model = model.to(device)

    # Loss function for multi-class segmentation
    # Add class weighting if dataset is imbalanced
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001, weight_decay=1e-4)

    # Train model
    train_model(
        model, train_loader, test_loader, criterion, optimizer, device, num_epochs=30
    )

    # Export to ONNX
    dummy_input = torch.randn(1, 3, img_size, img_size).to(device)
    torch.onnx.export(
        model,
        dummy_input,
        "results/segmentation_model.onnx",
        export_params=True,
        opset_version=11,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
    )
    print("Model exported to ONNX format")


if __name__ == "__main__":
    main()
