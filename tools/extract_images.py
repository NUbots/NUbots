#!/usr/bin/env python3

try:

    import os
    import json

    def register(command):

        # Install help
        command.help = "Decode an nbs file into a series of json objects"

        # Drone arguments
        command.add_argument("path", metavar="path", help="The nbs file to extract the compressed images from")

    def run(path, **kwargs):

        # Import the nbs decoder
        import re
        from util import nbs_decoder
        from google.protobuf.json_format import MessageToJson

        output_path = os.path.splitext(path)[0]
        os.makedirs(output_path, exist_ok=True)

        count = 0
        for type_name, timestamp, msg in nbs_decoder.decode(path):
            if type_name == "message.output.CompressedImage":
                with open(os.path.join(output_path, "img{:08d}.jpg".format(count)), "wb") as f:
                    f.write(msg.data)
                with open(os.path.join(output_path, "meta{:08d}.json".format(count)), "w") as f:
                    json.dump(
                        {
                            "Hcw": [
                                [msg.Hcw.x.x, msg.Hcw.x.y, msg.Hcw.x.z, msg.Hcw.x.t],
                                [msg.Hcw.y.x, msg.Hcw.y.y, msg.Hcw.y.z, msg.Hcw.y.t],
                                [msg.Hcw.z.x, msg.Hcw.z.y, msg.Hcw.z.z, msg.Hcw.z.t],
                                [msg.Hcw.t.x, msg.Hcw.t.y, msg.Hcw.t.z, msg.Hcw.t.t],
                            ],
                            "lens": {
                                "projection": msg.lens.projection,
                                "focal_length": msg.lens.focal_length,
                                "centre": [0, 0],
                                "fov": msg.lens.fov.x,
                            },
                        },
                        f,
                    )
                count += 1


except:
    print("Unable to load decode tool")

    def register(command):
        pass

    def run(**kwargs):
        pass
