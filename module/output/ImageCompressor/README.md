ImageCompressor
===============

## Description
Compresses images into JPEG format of specified quality, using a variety of compressors including `VAAPI` and `TurboJPEG`.
It will use compressors in order until it finds a free one to compress the image.
In the event that no compressors are currently free it will drop the image.

## Usage
Triggers on `message::input::Image`
Configuration from `ImageCompressor.yaml`

## Emits
`message::output::CompressedImage`

## Dependencies
TurboJPEG
libva (vaapi)
intel media driver
