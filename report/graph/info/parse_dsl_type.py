#!/usr/bin/python
import pdb
def parse_dsl_type(dsl, binder_args):

    el = dsl

    # Look through our tree finding known DSL words to parse
    if el[0] == 'NUClear':
        el = el[1:]
        if el[0] == 'dsl':
            el = el[1:]
            if el[0] == 'word':
                el = el[1:]

                # Types
                if el[0] == 'Trigger':
                    return {
                        'execution':  [{ 'scope': 'type', 'value': v, 'modifiers': {} } for v in el[1]],
                        'input_data': [{ 'scope': 'type', 'value': v, 'modifiers': {} } for v in el[1]]
                    }
                elif el[0] == 'With':
                    return {
                        'input_data': [{ 'scope': 'type', 'value': v, 'modifiers': {} } for v in el[1]]
                    }
                elif el[0] == 'IO':
                    return {
                        'execution':  [{ 'scope': 'io', 'value': None, 'modifiers': {} }],
                        'input_data': [{ 'scope': 'io', 'value': None, 'modifiers': {} }]
                    }

                # Type Modifiers
                elif el[0] == 'Optional':
                    v = parse_dsl_type(el[1][0], binder_args)
                    if 'input_data' in v:
                        for i in v['input_data']:
                            i['modifiers']['optional'] = True
                    return v
                elif el[0] == 'Last':
                    v = parse_dsl_type(el[1][1], binder_args)
                    if 'input_data' in v:
                        for i in v['input_data']:
                            i['modifiers']['last'] = el[1][0]
                    return v

                # Global Events
                elif el[0] == 'Startup':
                    return {
                        'execution':  [{ 'scope': 'system_event', 'value': 'Startup', 'modifiers': {} }],
                    }
                elif el[0] == 'Shutdown':
                    return {
                        'execution':  [{ 'scope': 'system_event', 'value': 'Shutdown', 'modifiers': {} }],
                    }

                # Timing
                elif el[0] == 'Always':
                    return {
                        'execution':  [{ 'scope': 'always', 'value': True, 'modifiers': {} }],
                    }
                elif el[0] == 'Every':

                    ticks = float(''.join(c for c in el[1][0][0] if c.isdigit()))

                    # If it's a standard duration one
                    if el[1][1][2] == 'duration':
                        # Get our components
                        ratio = el[1][1][-1][-1][-1]
                        num = float(''.join(c for c in ratio[0][0] if c.isdigit()))
                        den = float(''.join(c for c in ratio[1][0] if c.isdigit()))

                        timing = ticks * (num / den)

                    # If we are calculating a per
                    elif el[1][1][3] == 'Per':
                        ratio = el[1][1][4][0][-1][-1][-1]
                        num = float(''.join(c for c in ratio[0][0] if c.isdigit()))
                        den = float(''.join(c for c in ratio[1][0] if c.isdigit()))

                        timing = num / (ticks * den)

                    return {
                        'execution':  [{ 'scope': 'every', 'value': timing, 'modifiers': {} }],
                    }

                # Network
                elif el[0] == 'Network':
                    return {
                        'execution':  [{ 'scope': 'network', 'value': v, 'modifiers': {} } for v in el[1]],
                        'input_data': [{ 'scope': 'network', 'value': v, 'modifiers': {} } for v in el[1]]
                    }
                elif el[0] == 'UDP':
                    if len(el) == 1:
                        # Plain UDP
                        return {
                            'execution':  [{ 'scope': 'udp', 'value': None, 'modifiers': {} }],
                            'input_data': [{ 'scope': 'udp', 'value': None, 'modifiers': {} }]
                        }
                    elif el[1] == 'Broadcast':
                        # UDP broadcast
                        return {
                            'execution':  [{ 'scope': 'udp_broadcast', 'value': None, 'modifiers': {} }],
                            'input_data': [{ 'scope': 'udp_broadcast', 'value': None, 'modifiers': {} }]
                        }
                    elif el[1] == 'Multicast':
                        # UDP multicast
                        return {
                            'execution':  [{ 'scope': 'udp_multicast', 'value': None, 'modifiers': {} }],
                            'input_data': [{ 'scope': 'udp_multicast', 'value': None, 'modifiers': {} }]
                        }
                    else:
                        raise ValueError('Unknown DSL words', word)

                elif el[0] == 'TCP':
                    return {
                        'execution':  [{ 'scope': 'tcp', 'value': None, 'modifiers': {} }],
                        'input_data': [{ 'scope': 'tcp', 'value': None, 'modifiers': {} }]
                    }

                # Modifiers
                elif el[0] == 'Sync':
                    return { 'modifiers': { 'sync': el[1][0] } }
                elif el[0] == 'Single':
                    return { 'modifiers': { 'single': True } }
                elif el[0] == 'Priority':
                    return { 'modifiers': { 'priority': el[1] } }
                else:
                    raise ValueError('Unknown DSL words', word)
            else:
                raise ValueError('Unknown DSL words', word)
        else:
            raise ValueError('Unknown DSL words', word)

    elif el[0] == 'messages':
        el = el[1:]
        if el[0] == 'support':
            el = el[1:]
            if el[0] == 'Configuration':

                # The binder args contain the yaml file
                return {
                    'execution':  [{ 'scope': 'configuration', 'value': binder_args[0][0], 'modifiers': {} }],
                    'input_data': [{ 'scope': 'configuration', 'value': binder_args[0][0], 'modifiers': {} }]
                }
            elif el[0] == 'FileWatch':

                # The binder args contain the string argument
                return {
                    'execution':  [{ 'scope': 'file_watch', 'value': binder_args[0][0], 'modifiers': {} }],
                    'input_data': [{ 'scope': 'file_watch', 'value': binder_args[0][0], 'modifiers': {} }]
                }
            else:
                raise ValueError('Unknown DSL words', word)
        else:
            raise ValueError('Unknown DSL words', word)
    else:
        raise ValueError('Unknown DSL words', word)
