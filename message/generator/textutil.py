import textwrap


def indent(str, len=4):
    return '\n'.join([(' ' * len) + l for l in str.splitlines()])


def dedent(str):
    return textwrap.dedent(str)


def to_camel_case(snake_str):
    components = snake_str.split('_')
    # We capitalize the first letter of each component except the first one
    # with the 'title' method and join them together.
    return components[0] + "".join(x.title() for x in components[1:])
