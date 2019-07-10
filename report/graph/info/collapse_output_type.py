#!/usr/bin/python
import pdb


def collapse_output_type(output):

    # We will fill this with our new list of children
    newChildren = []

    # Loop through our outputs children
    for child in output["children"]:

        # Collapse the child
        child = collapse_output_type(child)

        # If our child is the same as us steal it's children
        if child["type"] == output["type"] and child["scopes"] == output["scopes"]:
            newChildren.extend(child["children"])

        # This isn't the same as us, add it (but it should be collapsed too)
        else:
            newChildren.append(child)

    # Our new children are now our children
    output["children"] = newChildren

    return output
