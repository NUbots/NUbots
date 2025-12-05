# Comparator

Module that calculates the sum of all numbers from k to n

## Description

This module calculates the sum of all integers from k to n using messages sent between it and a Summer module, which performs the actual sums.

## Usage

Include this module in your role, as well as the Summer module.
Modify k and n in the config file for your purposes.

## Consumes

- `message::onboarding::SumInt::SumResult` containing the resultant integer.

## Emits

- `message::onboarding::SumInt::SumInput` containing the integers to be summed.

## Dependencies

- module `onboarding::SumInt::Summer` to sum the numbers
