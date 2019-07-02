#!/usr/bin/python


def parse_output_type(output):

    o = []

    for scope in output["scopes"]:
        if scope == "Local":
            o.append({"scope": "type", "type": output["type"], "modifiers": {}})
            pass
        elif scope == "Direct":
            o.append({"scope": "type", "type": output["type"], "modifiers": {"direct": True}})
        elif scope == "Initialize":
            o.append({"scope": "type", "type": output["type"], "modifiers": {"initialize": True}})
        elif scope == "Network":
            o.append({"scope": "network", "type": output["type"], "modifiers": {}})
        elif scope == "UDP":
            o.append({"scope": "udp", "type": output["type"], "modifiers": {}})

    # Process all this output's children
    for child in output["children"]:
        o.extend(parse_output_type(child))

    return o
