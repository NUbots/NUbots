# GPT

## Description

A module that integrates with OpenAI's GPT-3.5 model to send text prompts and receive generated responses.

## Usage

Include this module to allow your system to interact with OpenAI's GPT for natural language processing tasks.

## Consumes

- `message::skill::GPTRequest`: A request containing text that needs to be processed by GPT.

## Emits

`message::skill::Say`: A Task requesting to speak using the output of the GPT model based on the input request.

## Dependencies

- **nlohmann::json**: For JSON data serialization and deserialization.
- **utility::openai::openai**: A utility to interface with the OpenAI API.
