#!/usr/bin/env python3

import os
import queue
import threading
from subprocess import DEVNULL, PIPE, Popen, call

from .fourcc import fourcc


def pix_fmt_from_fourcc(code):
    return {
        fourcc("BGGR"): "bayer_bggr8",
        fourcc("RGGB"): "bayer_rggb8",
        fourcc("GRBG"): "bayer_grbg8",
        fourcc("GBRG"): "bayer_gbrg8",
        fourcc("RGBA"): "rgba",
        fourcc("RGB3"): "rgb24",
        fourcc("RGB8"): "rgb24",
        fourcc("BGRA"): "bgra",
        fourcc("BGR3"): "bgr24",
        fourcc("BGR8"): "bgr24",
        fourcc("GRAY"): "gray",
        fourcc("GREY"): "gray",
        fourcc("Y8  "): "gray",
    }[code]


class Recorder:
    def __init__(self, output_path, dimensions, fourcc, encoder, bitrate, buffer_size=100):

        self.timecode_path = "{}_timecode.txt".format(os.path.splitext(output_path)[0])
        self.video_path = output_path
        self.buffer_size = buffer_size

        self.timecode = open(self.timecode_path, "w")
        self.frames = []
        self.start_time = None

        self.encoder = Popen(
            [
                "ffmpeg",
                "-hide_banner",
                "-y",
                "-f",
                "rawvideo",
                "-c:v",
                "rawvideo",
                "-pix_fmt",
                pix_fmt_from_fourcc(fourcc),
                "-s",
                "{}x{}".format(dimensions[1], dimensions[0]),
                "-i",
                "-",
                "-c:v",
                encoder,
                "-preset",
                "slow",
                "-profile:v",
                "high444" if encoder == "libx264" else "high",
                "-b:v",
                bitrate,
                self.video_path,
            ],
            stdin=PIPE,
            stdout=DEVNULL,
            stderr=DEVNULL,
        )

        self.queue = queue.Queue(10)
        self.thread = threading.Thread(target=self.process)
        self.thread.start()

    def _write_frame(self):

        # Sort the packets by timestamp and find the smallest one
        self.frames.sort(key=lambda m: m["timestamp"])
        msg = self.frames.pop(0)

        # Our offset start time is the smallest timestamp we have seen
        self.start_time = msg["timestamp"] if self.start_time is None else min(self.start_time, msg["timestamp"])

        # Calculate our timecode time and write it to the file
        self.timecode.write(
            "{}\n".format(
                1e3 * (msg["timestamp"][0] - self.start_time[0]) + 1e-6 * (msg["timestamp"][1] - self.start_time[1])
            )
        )

        self.encoder.stdin.write(msg["image"])

    def process(self):
        while True:
            item = self.queue.get()

            # If we get none, drain the frames and finish
            if item is None:
                while len(self.frames) > 0:
                    self._write_frame()
                return

            # Add this item
            self.frames.append(item)

            if len(self.frames) > self.buffer_size:
                self._write_frame()

    def encode(self, item):
        self.queue.put(item)

    def close(self):

        # Send none to close the thread
        self.queue.put(None)
        self.thread.join()

        # Close everything
        self.timecode.close()
        self.encoder.stdin.close()
        self.encoder.wait()

        # Fix the timecodes
        try:
            call(["mp4fpsmod", "-i", "-t", self.timecode_path, self.video_path])
        except FileNotFoundError:
            print("mp4fpsmod is not installed in the path")
