def type_to_string(type_list):

    # Remove the allocator from vectors to make them easier to read
    if len(type_list) > 2 and type_list[:2] == ["std", "vector"]:
        type_list = type_list[:2] + [[type_list[2][0]]]

    # Parse Every into sanity
    if len(type_list) > 4 and type_list[:4] == ["NUClear", "dsl", "word", "Every"]:

        # Get the number of ticks we are doing
        ticks = int(type_list[4][0][0])

        # Extract our period information
        period = type_list[4][1]
        is_per = len(type_list[4][1]) > 4 and period[:4] == ["NUClear", "dsl", "word", "Per"]
        if is_per:
            period = period[4][0]
        period = period[3][-1][-1]

        period = ["std", "chrono"] + {
            1e-9: ["nanoseconds"],
            1e-6: ["microseconds"],
            1e-3: ["milliseconds"],
            1: ["seconds"],
            60: ["minutes"],
            3600: ["hours"],
        }[float(period[0][0][:-2]) / float(period[1][0][:-2])]

        if is_per:
            type_list = type_list[:4] + [[[str(ticks)], ["NUClear", "dsl", "word", "Per", [period]]]]
        else:
            type_list = type_list[:4] + [[[str(ticks)], period]]
            pass

    flat_list = []
    short_type = False
    previous = type_list[0]

    for item in type_list:
        if isinstance(item, basestring) and (short_type or (item[0].isupper() and item != "NUClear")):
            short_type = True
            flat_list.append(item)
        elif not isinstance(item, basestring):
            # If we didn't have a type yet we must use the last one
            if not short_type:
                flat_list.append(previous)
                short_type = True
            flat_list.append("<{}>".format(", ".join([type_to_string(subitem) for subitem in item])))
        else:
            previous = item

    if not short_type:
        flat_list.append(previous)

    out = "::".join(flat_list)
    out = out.replace("::<", "<")
    return out
