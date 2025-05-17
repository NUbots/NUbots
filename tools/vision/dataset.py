import os
import random

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset


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

    def find_matching_mask(self, img_name):
        """Find the matching mask for an image with different possible naming patterns."""
        base_name, ext = os.path.splitext(img_name)

        # Try all possible mask naming patterns
        potential_mask_names = [
            # Same filename
            img_name,
            # Different extensions
            f"{base_name}.png",
            f"{base_name}.jpg",
            # With _mask suffix
            f"{base_name}_mask{ext}",
            f"{base_name}_mask.png",
            f"{base_name}_mask.jpg"
        ]

        for mask_name in potential_mask_names:
            mask_path = os.path.join(self.mask_dir, mask_name)
            if os.path.exists(mask_path):
                return mask_path

        # If no matching mask is found
        raise FileNotFoundError(f"No matching mask found for image: {img_name}")

    def __getitem__(self, idx):
        img_name = self.images[idx]
        img_path = os.path.join(self.img_dir, img_name)

        # Find the corresponding mask file
        try:
            mask_path = self.find_matching_mask(img_name)
        except FileNotFoundError as e:
            print(e)
            # Create a black mask as fallback (optional)
            image = Image.open(img_path).convert("RGB")
            image = image.resize((self.img_size, self.img_size), Image.BILINEAR)
            mask = Image.new("RGB", (self.img_size, self.img_size), (0, 0, 0))

            if self.transform:
                image = self.transform(image)
                mask_tensor = torch.zeros((self.img_size, self.img_size), dtype=torch.long)
                return image, mask_tensor
            return image, mask

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
