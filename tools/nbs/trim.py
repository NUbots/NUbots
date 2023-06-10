#!/usr/bin/env python3

import argparse
from datetime import timedelta
from pathlib import Path

from pytimeparse import parse
from termcolor import colored
from tqdm import tqdm

from utility.nbs import Encoder, LinearDecoder


def register(command):
    command.description = "Merge a list of nbs files and trim to the specified start and end times."

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to merge and trim.")
    command.add_argument(
        "-s",
        "--start",
        default="+0s",
        help="""
        The start of the trimmed nbs files relative to the start or end of the file.
        Format is a +/- followed by any combination of 'Xh' for hours, 'Xm' for minutes,
        or 'Xs' for seconds. E.g. '-s=+10m10s' means the start of the merged nbs file plus 10
        minutes and 10 seconds, and '-s=-1h' means the end of the merged nbs file minus 1 hour.
        Defaults to '+0s'.""",
    )
    command.add_argument(
        "-e",
        "--end",
        default="-0s",
        help="""The end of the trimmed nbs file relative to the start or end of the merged nbs file.
        Format is the same as START. Defaults to '-0s'.""",
    )
    command.add_argument(
        "-o",
        "--output",
        help="""The trimmed nbs file. Defaults to the input file name with appended trim boundary
        strings. If multiple files are used, adds 'merged' to the end of the file name.""",
    )
    command.add_argument(
        "--message-timestamp",
        "-t",
        dest="use_message_timestamp",
        action="store_true",
        help="""If available use the messages `.timestamp` field as the time of the message""",
    )
    command.add_argument(
        "--show-info",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show info messages from this tool. Default is true.",
    )


def bold(text):
    return colored(text, attrs=["bold"])


def run(files, use_message_timestamp, start, end, output, show_info=True, **kwargs):
    if output is None:
        file_no_ext = Path(files[0]).with_suffix("")
        ext = "nbs"
        if len(files) > 1:
            output = f"{file_no_ext}_{start}_{end}_merged.{ext}"
        else:
            output = f"{file_no_ext}_{start}_{end}.{ext}"

    decoder = LinearDecoder(*files, show_progress=show_info)

    # Convert start and end offsets into nanoseconds
    start_offset = parse(start) * 1e9 if start is not None else 0
    end_offset = parse(end) * 1e9 if end is not None else 0

    if use_message_timestamp:
        # Get the first and last packet in nanoseconds
        first_packet = decoder[0]
        last_packet = decoder[-1]

        if hasattr(first_packet.msg, "timestamp") and hasattr(last_packet.msg, "timestamp"):
            # Get the first/last timestamp in nanoseconds
            first_timestamp = first_packet.msg.timestamp.seconds * 1.0e9 + first_packet.msg.timestamp.nanos
            last_timestamp = last_packet.msg.timestamp.seconds * 1.0e9 + last_packet.msg.timestamp.nanos
        else:
            # Get the first and last packet index timestamps in microseconds
            first_timestamp, last_timestamp = decoder.index["timestamp"][[0, -1]]
    else:
        # Get the first and last packet index timestamps in microseconds
        first_timestamp, last_timestamp = decoder.index["timestamp"][[0, -1]]

    # Get the start timestamp in microseconds of trimmed file
    start_timestamp = first_timestamp + start_offset if start_offset >= 0 else last_timestamp + start_offset

    # Get the end timestamp in microseconds of trimmed file
    end_timestamp = first_timestamp + end_offset if end_offset > 0 else last_timestamp + end_offset

    if show_info:
        print("")
        print(f"   First packet index timestamp: {bold(f'{first_timestamp} ns')}")
        print(f"        Start of trim timestamp: {bold(f'{start_timestamp} ns')}")
        print(f"          End of trim timestamp: {bold(f'{end_timestamp} ns')}")
        print(f"    Last packet index timestamp: {bold(f'{last_timestamp} ns')}")
        print("")
        print(
            f"     Original length (hh:mm:ss): {bold(timedelta(microseconds=(last_timestamp - first_timestamp)/1000))}"
        )
        print(
            f"      Trimmed length (hh:mm:ss): {bold(timedelta(microseconds=(end_timestamp - start_timestamp)/1000))}"
        )
        print("")
        print(f"    Input file: {bold(files)}")
        print(f"   Output file: {bold(output)}")
        print("")

    if start_timestamp == first_timestamp and end_timestamp == last_timestamp:
        print("No output created as the start and end trim times are the same as the start and end of the input file.")
        exit()
    if end_timestamp <= start_timestamp:
        print("Error: End of trim can't be earlier than or equal to start of trim.")
        exit(1)

    with Encoder(output) as out:
        packets = tqdm(decoder, unit="packet", unit_scale=True, dynamic_ncols=True) if show_info else decoder
        for packet in packets:
            # Only output packets that are within the packet index timestamp range
            ts = (
                packet.msg.timestamp.seconds * 1.0e9 + packet.msg.timestamp.nanos
                if use_message_timestamp
                else packet.index_timestamp
            )

            if start_timestamp <= ts <= end_timestamp:
                out.write(int(ts) if use_message_timestamp else packet.emit_timestamp, packet.msg)
