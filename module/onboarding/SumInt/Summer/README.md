# Summer

Simple module that adds two numbers together.

## Description

This module consumes a message containing two integers (uint64), sums them, and emits a message with the result.

## Usage

Include this module in your role, and emit a SumInput message containing the integers you NEED to sum.
Then wait for this module to emit a SumResult message which contains the result of the sum.

## Consumes

- `message::onboarding::SumInt::SumInput` containing the integers to be summed.

## Emits

- `message::onboarding::SumInt::SumResult` containing the resultant integer.

## Dependencies

None.
