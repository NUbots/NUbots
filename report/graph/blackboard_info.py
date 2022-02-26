#!/usr/bin/python

import json
import os
import sys

from PIL import Image

files = os.listdir(sys.argv[1])
type_files = [f for f in files if f.endswith("types.json")]
info_files = [f for f in files if f.endswith("info.json")]

roles = []

for filename in type_files:
    with open(sys.argv[1] + "/" + filename, "r") as f:
        data = json.load(f)

        messages = []

        for v in data["inputs"]:
            t = {"scope": v["scope"], "type": v["type"]}
            if t["scope"] == "type" and t not in messages:
                messages.append(t)

        for v in data["outputs"]:
            t = {"scope": v["scope"], "type": v["type"]}
            if t["scope"] == "type" and t not in messages:
                messages.append(t)

    roles.append(messages)

complexity_points = set()
for filename in info_files:

    val = 0

    with open(sys.argv[1] + "/" + filename, "r") as f:
        data = json.load(f)

        for d in data["message_lmb_complexity"]:
            complexity_points.add((d[0], d[1], d[2]))
            val += d[3]

    print("{}\t{}".format(data["number_of_modules"], val))

for p in complexity_points:
    print("{}\t{}\t{}".format(p[0], p[1], p[2]))

usage = {}
maximum = 0

for r in roles:
    for m in r:
        usage[str(m)] = usage.get(str(m), 0) + 1
        maximum = max(usage[str(m)], maximum)

for k in usage:
    usage[k] = float(usage[k]) / float(maximum)

points = sorted([usage[k] for k in usage], reverse=True)

img = Image.new("RGB", (11, 11), "black")
img2 = Image.new("RGB", (166, 166), "black")
pixels = img.load()
pixels2 = img2.load()

for i in xrange(0, len(points)):
    for x in xrange(1, 15):
        for y in xrange(1, 15):
            pixels2[int((i % 11) * 15 + x), int((i / 11) * 15 + y)] = (
                int(points[i] * 255),
                int(points[i] * 255),
                int(points[i] * 255),
            )

for i in xrange(0, len(points)):
    pixels[int(i % 11), int(i / 11)] = (int(points[i] * 255), int(points[i] * 255), int(points[i] * 255))

img.save("out.png")
img2.save("out2.png")
