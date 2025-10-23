# Judge

## Description

Compares Accumulator output to expected result

## Usage

Configuration from `Judge.yaml` (for log level settings)

## Consumes

- `message::onboarding::FinalAnswer` - The computed sum result from the Accumulator module

## Emits

- `message::skill::Say` (as Task) - Text-to-speech command with head nodding when answer is

## Dependencies

- `extension::behaviour::BehaviourReactor` - Provides Task emission capabilities
- `message::onboarding::FinalAnswer` - Message type for receiving computation results
- `message::skill::Say` - Message type for robot speech and nodding
- `skill::Say` module - Must be included in role file to process Say messages
