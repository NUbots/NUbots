#!/usr/bin/python
import pdb


def parse_dsl_type(dsl, binder_args):

    el = dsl

    # Look through our tree finding known DSL words to parse
    if el[0] == "NUClear":
        el = el[1:]
        if el[0] == "dsl":
            el = el[1:]
            if el[0] == "word":
                el = el[1:]

                # Types
                if el[0] == "Trigger":
                    return {
                        "input_data": [
                            {"scope": "type", "type": v, "modifiers": {"execution": True, "data": True}} for v in el[1]
                        ]
                    }
                elif el[0] == "With":
                    return {"input_data": [{"scope": "type", "type": v, "modifiers": {"data": True}} for v in el[1]]}
                elif el[0] == "IO":
                    return {
                        "input_data": [{"scope": "io", "type": None, "modifiers": {"execution": True, "data": True}}]
                    }

                # Type Modifiers
                elif el[0] == "Optional":
                    v = parse_dsl_type(el[1][0], binder_args)
                    if "input_data" in v:
                        for i in v["input_data"]:
                            i["modifiers"]["optional"] = True
                    return v
                elif el[0] == "Last":
                    v = parse_dsl_type(el[1][1], binder_args)
                    if "input_data" in v:
                        for i in v["input_data"]:
                            i["modifiers"]["last"] = el[1][0]
                    return v

                # Global Events
                elif el[0] == "Startup":
                    return {
                        "input_data": [{"scope": "system_event", "type": "Startup", "modifiers": {"execution": True}}]
                    }
                elif el[0] == "Shutdown":
                    return {
                        "input_data": [{"scope": "system_event", "type": "Shutdown", "modifiers": {"execution": True}}]
                    }

                # Timing
                elif el[0] == "Always":
                    return {"input_data": [{"scope": "always", "type": True, "modifiers": {"execution": True}}]}
                elif el[0] == "Every":

                    ticks = float("".join(c for c in el[1][0][0] if c.isdigit()))

                    # If it's a standard duration one
                    if el[1][1][2] == "duration":
                        # Get our components
                        ratio = el[1][1][-1][-1][-1]
                        num = float("".join(c for c in ratio[0][0] if c.isdigit()))
                        den = float("".join(c for c in ratio[1][0] if c.isdigit()))

                        timing = ticks * (num / den)

                    # If we are calculating a per
                    elif el[1][1][3] == "Per":
                        ratio = el[1][1][4][0][-1][-1][-1]
                        num = float("".join(c for c in ratio[0][0] if c.isdigit()))
                        den = float("".join(c for c in ratio[1][0] if c.isdigit()))

                        timing = num / (ticks * den)

                    return {"input_data": [{"scope": "every", "type": timing, "modifiers": {"execution": True}}]}

                # Network
                elif el[0] == "Network":
                    return {
                        "input_data": [
                            {"scope": "network", "type": v, "modifiers": {"execution": True, "data": True}}
                            for v in el[1]
                        ]
                    }
                elif el[0] == "UDP":
                    if len(el) == 1:
                        # Plain UDP
                        return {
                            "input_data": [
                                {"scope": "udp", "type": None, "modifiers": {"execution": True, "data": True}}
                            ]
                        }
                    elif el[1] == "Broadcast":
                        # UDP broadcast
                        return {
                            "input_data": [
                                {"scope": "udp_broadcast", "type": None, "modifiers": {"execution": True, "data": True}}
                            ]
                        }
                    elif el[1] == "Multicast":
                        # UDP multicast
                        return {
                            "input_data": [
                                {"scope": "udp_multicast", "type": None, "modifiers": {"execution": True, "data": True}}
                            ]
                        }
                    else:
                        raise ValueError("Unknown DSL words", word)

                elif el[0] == "TCP":
                    return {
                        "input_data": [{"scope": "tcp", "type": None, "modifiers": {"execution": True, "data": True}}]
                    }

                # Modifiers
                elif el[0] == "Sync":
                    return {"modifiers": {"sync": el[1][0]}}
                elif el[0] == "Single":
                    return {"modifiers": {"single": True}}
                elif el[0] == "Priority":
                    return {"modifiers": {"priority": el[1]}}
                else:
                    raise ValueError("Unknown DSL words", word)
            else:
                raise ValueError("Unknown DSL words", word)
        else:
            raise ValueError("Unknown DSL words", word)

    elif el[0] == "messages":
        el = el[1:]
        if el[0] == "support":
            el = el[1:]
            if el[0] == "Configuration":

                # The binder args contain the yaml file
                return {
                    "input_data": [
                        {
                            "scope": "configuration",
                            "type": binder_args[0][0],
                            "modifiers": {"execution": True, "data": True},
                        }
                    ]
                }
            elif el[0] == "FileWatch":

                # The binder args contain the string argument
                return {
                    "input_data": [
                        {
                            "scope": "file_watch",
                            "type": binder_args[0][0],
                            "modifiers": {"execution": True, "data": True},
                        }
                    ]
                }
            else:
                raise ValueError("Unknown DSL words", word)
        else:
            raise ValueError("Unknown DSL words", word)
    else:
        raise ValueError("Unknown DSL words", word)
