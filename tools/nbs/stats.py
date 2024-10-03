#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import datetime
import json
import os

import numpy as np
from si_prefix import si_format
from tqdm import tqdm

from utility.nbs import LinearDecoder


def register(command):
    command.description = "Decode an nbs file into a series of file statistics"

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to examine")
    command.add_argument(
        "--message-timestamp",
        "-t",
        dest="use_message_timestamp",
        action="store_true",
        help="If available use the messages `.timestamp` field as the time of the message",
    )


def sum_by_group(values, groups):
    values = np.cumsum(values)
    index = np.ones(len(groups), "bool")
    index[:-1] = groups[1:] != groups[:-1]
    values = values[index]
    groups = groups[index]
    values[1:] = values[1:] - values[:-1]
    return values, groups


def compute_statistics(group, data):
    total_packets = len(data)
    total_bytes = np.sum(data["bytes"])
    total_time = np.max(data["timestamp"]) - np.min(data["timestamp"])

    if total_time == 0:
        return {
            "total_packets": total_packets,
            "total_bytes": total_bytes,
            "packet_rate": np.nan,
            "bytes_rate": np.nan,
            "minmax_packet_rate": [np.nan, np.nan],
            "minmax_bytes_rate": [np.nan, np.nan],
            "minmax_timestamp": [np.min(data["timestamp"]), np.max(data["timestamp"])],
        }

    # Group elements with the same timestamp together to avoid a divide by 0 error
    local_packet_count, ts = sum_by_group(np.ones_like(data["timestamp"]), data["timestamp"])
    local_bytes_count, ts = sum_by_group(data["bytes"], data["timestamp"])

    # Work out the local packet and bytes rate
    delta = ts[1:] - ts[:-1]
    local_packet_rate = local_packet_count[:-1] / delta
    local_bytes_rate = local_bytes_count[:-1] / delta

    # Find the lowest and highest local packet rates encountered
    minmax_packet_rate = [np.nanmin(local_packet_rate), np.nanmax(local_packet_rate)]
    minmax_bytes_rate = [np.nanmin(local_bytes_rate), np.nanmax(local_bytes_rate)]
    return {
        "total_packets": total_packets,
        "total_bytes": total_bytes,
        "packet_rate": total_packets / total_time,
        "bytes_rate": total_bytes / total_time,
        "minmax_packet_rate": minmax_packet_rate,
        "minmax_bytes_rate": minmax_bytes_rate,
        "minmax_timestamp": [np.min(data["timestamp"]), np.max(data["timestamp"])],
    }


def stats_to_string(stats):
    if not (np.isnan(stats["packet_rate"])):
        return "{packet_rate}Hz ({packet_rate_l}Hz ‒ {packet_rate_u}Hz) ({packets} total)\n{bytes_rate}B/s ({bytes_rate_l}B/s ‒ {bytes_rate_u}B/s) ({bytes}B total)\n{min_timestamp} ‒ {max_timestamp}".format(
            packets=stats["total_packets"],
            bytes=si_format(stats["total_bytes"]),
            packet_rate=si_format(stats["packet_rate"] * 1e9),
            bytes_rate=si_format(stats["bytes_rate"] * 1e9),
            packet_rate_l=si_format(stats["minmax_packet_rate"][0] * 1e9),
            packet_rate_u=si_format(stats["minmax_packet_rate"][1] * 1e9),
            bytes_rate_l=si_format(stats["minmax_bytes_rate"][0] * 1e9),
            bytes_rate_u=si_format(stats["minmax_bytes_rate"][1] * 1e9),
            min_timestamp=datetime.datetime.fromtimestamp(stats["minmax_timestamp"][0] * 1e-9).isoformat(),
            max_timestamp=datetime.datetime.fromtimestamp(stats["minmax_timestamp"][1] * 1e-9).isoformat(),
        )
    else:
        return "({packets} total)\n({bytes}B total)\n{timestamp}".format(
            packets=stats["total_packets"],
            bytes=si_format(stats["total_bytes"]),
            timestamp=datetime.datetime.fromtimestamp(stats["minmax_timestamp"][0] * 1e-9).isoformat(),
        )


def indent(in_string, count):
    return "\n".join(["{}{}".format(" " * count, s) for s in in_string.split("\n")])


def run(files, use_message_timestamp, **kwargs):
    data = []
    max_cat_len = 0
    max_subcat_len = 0

    for packet in tqdm(
        LinearDecoder(*files, show_progress=True),
        unit="packet",
        unit_scale=True,
        dynamic_ncols=True,
    ):
        # If our object has a timestamp field of its own, we can decide to use that instead of the nbs timestamp
        # This can sometimes give better rates but can also cause problems when the timebases aren't synchronized
        if use_message_timestamp and hasattr(packet.msg, "timestamp"):
            ts = packet.msg.timestamp.seconds * 1e9 + packet.msg.timestamp.nanos
        else:
            ts = packet.emit_timestamp * 1e3

        # Treat some objects specially as they have sub categories
        if packet.type.name in ("message.input.Image", "message.output.CompressedImage"):
            data.append((packet.type.name, packet.msg.name, ts, len(packet.raw_payload)))
        else:
            data.append((packet.type.name, "", ts, len(packet.raw_payload)))

        # Work out the largest name length so we can put it into numpy
        max_cat_len = max(max_cat_len, len(data[-1][0]))
        max_subcat_len = max(max_subcat_len, len(data[-1][1]))

    data = np.array(
        data,
        dtype=[
            ("category", "S{}".format(max_cat_len)),
            ("subcategory", "S{}".format(max_subcat_len)),
            ("timestamp", np.int64),
            ("bytes", np.int64),
        ],
    )
    data = np.sort(data, order="timestamp")
    categories = np.unique(data[["category", "subcategory"]])

    stats = {}
    global_stats = compute_statistics("Global", data)
    for c, sc in categories:
        # If we haven't seen this message type before process it
        if c.decode("utf-8") not in stats:
            stats[c.decode("utf-8")] = compute_statistics(c.decode("utf-8"), data[np.where(data["category"] == c)])

        if sc != b"":
            stats["{}#{}".format(c.decode("utf-8"), sc.decode("utf-8"))] = compute_statistics(
                "{}#{}".format(c.decode("utf-8"), sc.decode("utf-8")),
                data[np.where(np.logical_and(data["category"] == c, data["subcategory"] == sc))],
            )

    print("Global")
    print(indent(stats_to_string(global_stats), 2))
    print()
    for k, v in stats.items():
        if "#" in k:
            print(indent(k, 8))
            print(indent(stats_to_string(v), 10))
        else:
            print(indent(k, 4))
            print(indent(stats_to_string(v), 6))
        print()
