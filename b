#!/bin/bash
dir="$(dirname "$0")"
exec uv run --project "$dir" "$dir/nuclear/b.py" "$@"
