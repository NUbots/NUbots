# PlotJuggler

## Description

This module allows for sending data from NUbots to [PlotJuggler](https://www.plotjuggler.io/) for plotting in real time. It does this by listening for `DataPoint` messages (what we use for plotting in NUsight), transforming them to a format suitable for PlotJuggler, and sending them to PlotJuggler via UDP.


## Usage

To plot data from a role you're working on using PlotJuggler:

- [Install PlotJuggler](https://github.com/facontidavide/PlotJuggler#installation) on your computer.
- Launch PlotJuggler and start the UDP server. Take note of the IP address and port.
- Add `network::PlotJuggler` to the role you're working on
- Update the `PlotJuggler.yaml` config file with the PlotJuggler UDP server and enable forwarding of `DataPoint`s:
    - TODO: add screenshot
    -

## Dependencies

- `message::support::nusight::DataPoint` when debug waves are enabled for testing the connection to PlotJuggler
- JSON-formatted packets to PlotJuggler via UDP

## Emits

- `message::support::nusight::DataPoint` when debug waves are enabled for testing the connection to PlotJuggler
- JSON-formatted packets to PlotJuggler via UDP
