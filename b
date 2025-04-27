#!/bin/bash

# Determine which checksum command to use
if type sha256sum &> /dev/null; then
  checksum_cmd="sha256sum"
elif type shasum &> /dev/null; then
  checksum_cmd="shasum -a 256"
else
  echo "No suitable checksum command found."
  exit 1
fi

script_dir=$(dirname "$0")
poetry_env_path=$(poetry env info --path 2>/dev/null)

if [ -z "${poetry_env_path}" ] || ! ${checksum_cmd} --check --status "${poetry_env_path}/poetry.lock.sha256" 2>/dev/null; then
  poetry sync

  poetry_env_path=$(poetry env info --path 2>/dev/null)

  ${checksum_cmd} "${script_dir}/poetry.lock" > "${poetry_env_path}/poetry.lock.sha256"
fi

exec poetry run "${script_dir}/nuclear/b.py" "$@"
