# ImageDecompressor

## Description

Decompresses images that are in JPEG format to raw images, using a variety of decompressors.
It will use decompressors in order until it finds a free one to compress the image.
In the event that no compressors are currently free it will drop the image.

## Usage

Triggers on `message::output::CompressedImage`
Configuration from `ImageDecompressor.yaml`

## Emits

`message::input::Image`

## Dependencies

TurboJPEG
