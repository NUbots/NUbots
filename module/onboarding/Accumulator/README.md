# Accumulator

## Description

Performs iterative summation of integers from k=1 to n=10 without using loops. Receives iteration messages, adds the current value k to the running sum, and either continues the iteration or emits the final result

## Usage

Configuration from `Accumulator.yaml` (for log level settings)

## Consumes

- `message::onboarding::ComputeIteration`

## Emits

- `message::onboarding::ComputeIteration`
- `message::onboarding::FinalAnswer`

## Dependencies

- `message::onboarding::ComputeIteration` - Message type for iteration state
- `message::onboarding::FinalAnswer` - Message type for final result
- `onboarding::Starter` - Module that initiates the first ComputeIteration message
- `onboarding::Judge` - Module that validates the final answer
