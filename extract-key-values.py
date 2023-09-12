import argparse
import json
from fnmatch import fnmatchcase


# This function was intended to extract nested values with common parent keys
# for example data.servo.{alltheservos}.present_current
def extract_keys_with_wildcard(data, keys):
    extracted_dict = {}

    def extract_recursive(data, key):
        if isinstance(data, dict):
            for k, v in data.items():
                current_key = f"{key}.{k}" if key else k
                if fnmatchcase(current_key, key):
                    extracted_dict[current_key] = v
                if isinstance(v, (dict, list)):
                    extract_recursive(v, current_key)

    for key in keys:
        extract_recursive(data, key)

    return extracted_dict


def extract_keys(data, keys):
    extracted_key_values = {}
    for key in keys:
        key_parts = key.split(".")
        value = data
        try:
            for part in key_parts:
                value = value[part]
            extracted_key_values[key] = value
        except KeyError:
            extracted_key_values[key] = None
    return extracted_key_values


def process_records(json_lines, keys_to_extract):
    results = []
    for line in json_lines:
        data = json.loads(line)  # Parse each line as a JSON object
        extracted_key_values = extract_keys(data, keys_to_extract)
        results.append(extracted_key_values)
    return results


def main():
    parser = argparse.ArgumentParser(description="Extract specified keys from JSON data")
    parser.add_argument("json_file", help="Path to the JSON data file")
    parser.add_argument("config_file", help="Path to the configuration file")
    parser.add_argument("output_file", help="Name/Path of the new file")

    args = parser.parse_args()

    # Read JSON data file line by line
    with open(args.json_file, "r") as json_file:
        json_lines = json_file.readlines()

    # Config file (the values we want from the JSON file)
    with open(args.config_file, "r") as config_file:
        config = json.load(config_file)
        keys_to_extract = config.get("keys_to_extract", [])

    extracted_records = process_records(json_lines, keys_to_extract)

    with open(args.output_file, "w") as to_file:
        json.dump(extracted_records, to_file, indent=2)

    for i, extracted_key_values in enumerate(extracted_records):
        print(f"Record {i + 1}:")
        for key, value in zip(keys_to_extract, extracted_key_values):
            print(f"{key}: {value}")


if __name__ == "__main__":
    main()
