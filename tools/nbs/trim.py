#!/usr/bin/env python3

from datetime import timedelta
from pathlib import Path

from pytimeparse import parse
from termcolor import colored
from tqdm import tqdm

from utility.nbs import Encoder, LinearDecoder


def register(command):
    command.help = "Trim an nbs file to the specified start and end times. "

    # Command arguments
    command.add_argument("file", help="The nbs file to trim.")
    command.add_argument(
        "-s",
        "--start",
        default="+0s",
        help="""
        The start of the trimmed nbs file relative to the start or end of the file.
        Format is a +/- followed by any combination of 'Xh' for hours, 'Xm' for minutes,
        or 'Xs' for seconds. E.g. '-s=+10m10s' means the start of the nbs file plus 10
        minutes and 10 seconds, and '-s=-1h' means the end of the nbs file minus 1 hour.
        Defaults to '+0s'.""",
    )
    command.add_argument(
        "-e",
        "--end",
        default="-0s",
        help="""The end of the trimmed nbs file relative to the start or end of the file.
        Format is the same as START. Defaults to '-0s'.""",
    )
    command.add_argument(
        "-o",
        "--output",
        help="The trimmed nbs file. Defaults to the input file name with appended trim boundary strings.",
    )


def bold(text):
    return colored(text, attrs=["bold"])


def run(file, start, end, output, **kwargs):
    if output is None:
        file_no_ext = Path(file).with_suffix("")
        output = f"{file_no_ext}_{start}_{end}.nbs"

    decoder = LinearDecoder(file, show_progress=True)

    # Get the first and last packet index timestamps in microseconds
    first_timestamp, last_timestamp = decoder.index["timestamp"][[0, -1]]

    # Get the start timestamp in microseconds of trimmed file
    start_offset = parse(start) * 1000000 if start is not None else 0
    start_timestamp = first_timestamp + start_offset if start_offset >= 0 else last_timestamp + start_offset

    # Get the end timestamp in microseconds of trimmed file
    end_offset = parse(end) * 1000000 if end is not None else 0
    end_timestamp = first_timestamp + end_offset if end_offset > 0 else last_timestamp + end_offset

    print("")
    print(f"   First packet index timestamp: {bold(f'{first_timestamp} µs')}")
    print(f"        Start of trim timestamp: {bold(f'{start_timestamp} µs')}")
    print(f"          End of trim timestamp: {bold(f'{end_timestamp} µs')}")
    print(f"    Last packet index timestamp: {bold(f'{last_timestamp} µs')}")
    print("")
    print(f"     Original length (hh:mm:ss): {bold(timedelta(microseconds=int(last_timestamp - first_timestamp)))}")
    print(f"      Trimmed length (hh:mm:ss): {bold(timedelta(microseconds=int(end_timestamp - start_timestamp)))}")
    print("")
    print(f"    Input file: {bold(file)}")
    print(f"   Output file: {bold(output)}")
    print("")

    if start_timestamp == first_timestamp and end_timestamp == last_timestamp:
        print("No output created as the start and end trim times are the same as the start and end of the input file.")
        exit()
    if end_timestamp <= start_timestamp:
        print("Error: End of trim can't be earlier than or equal to start of trim.")
        exit(1)

    with Encoder(output) as out:
        for packet in tqdm(decoder, unit="packet", unit_scale=True, dynamic_ncols=True):
            # Only output packets that are within the packet index timestamp range
            if start_timestamp <= packet.index_timestamp <= end_timestamp:
                # Output the packet message with its emit timestamp
                out.write(packet.emit_timestamp, packet.msg)
