import argparse
import json


def extract_keys(data, keys):
    extracted_values = []
    for key in keys:
        key_parts = key.split(".")
        value = data
        try:
            for part in key_parts:
                value = value[part]
            extracted_values.append(value)
        except KeyError:
            extracted_values.append(None)
    return extracted_values


def main():
    parser = argparse.ArgumentParser(
        description="Extract specified keys from JSON data"
    )
    parser.add_argument("json_file", help="Path to the JSON data file")
    parser.add_argument("config_file", help="Path to the configuration file")

    args = parser.parse_args()

    # data input file
    with open(args.json_file, "r") as json_file:
        json_data = json.load(json_file)

    # config file (the values we want from the json file)
    with open(args.config_file, "r") as config_file:
        config = json.load(config_file)
        keys_to_extract = config.get("keys_to_extract", [])

    extracted_values = extract_keys(json_data, keys_to_extract)

    with open("newData.json", "w") as toFile:
        json.dump(extracted_values, toFile)

    for key, value in zip(keys_to_extract, extracted_values):
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
