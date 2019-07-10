NUsight
========

## Description

This module simply emits sine wave data points to NUsight. It can be used to quickly test the
connection between NUbots and NUsight.

## Usage

Include the module in a role, and it will automatically emit messages.

Also ensure that sending data points are enabled in `config/NUsight.yaml`

## Emits

* `message::support::nusight::DataPoint`
