#!/bin/bash
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"

# `nuclear/b.py` is a `uv run --script` entrypoint, but some shells (or GUI launchers)
# may not include common user-local bin paths (e.g. ~/.local/bin) in $PATH.
# Resolve `uv` explicitly to avoid /usr/bin/env failing to find it.
uv_bin=""

user_name="$(id -un)"
user_home=""
if user_home_line="$(getent passwd "${user_name}" 2>/dev/null)"; then
	user_home="$(printf '%s' "${user_home_line}" | cut -d: -f6)"
fi
if [[ -z "${user_home}" ]]; then
	user_home="${HOME:-}"
fi

if [[ -n "${NU_BOTS_UV:-}" && -x "${NU_BOTS_UV}" ]]; then
	uv_bin="${NU_BOTS_UV}"
elif [[ -x "${script_dir}/.venv/bin/uv" ]]; then
	uv_bin="${script_dir}/.venv/bin/uv"
elif [[ -n "${user_home}" && -x "${user_home}/.local/bin/uv" ]]; then
	uv_bin="${user_home}/.local/bin/uv"
elif [[ -n "${user_home}" && -x "${user_home}/.cargo/bin/uv" ]]; then
	uv_bin="${user_home}/.cargo/bin/uv"
elif command -v uv >/dev/null 2>&1; then
	uv_bin="$(command -v uv)"
else
	# Last resort: try `whereis` (may not find user-local installs on all distros)
	if whereis_out="$(whereis -b uv 2>/dev/null)"; then
		# whereis output: "uv: /path1 /path2 ..."; pick the first existing executable
		for candidate in ${whereis_out#uv:}; do
			if [[ -f "${candidate}" && -x "${candidate}" ]]; then
				uv_bin="${candidate}"
				break
			fi
		done
	fi
fi

if [[ -z "${uv_bin}" ]]; then
	# Fallback: allow running without uv (e.g. inside older prebuilt Docker images)
	if command -v python3 >/dev/null 2>&1; then
		exec python3 "${script_dir}/nuclear/b.py" "$@"
	elif command -v python >/dev/null 2>&1; then
		exec python "${script_dir}/nuclear/b.py" "$@"
	fi

	echo "ERROR: 'uv' was not found, and no python interpreter was available for fallback." >&2
	echo "  - Install uv: pip install --user uv (or: curl -LsSf https://astral.sh/uv/install.sh | sh)" >&2
	if [[ -n "${user_home}" ]]; then
		echo "  - Ensure: export PATH=\"${user_home}/.local/bin:$PATH\"" >&2
	else
		echo "  - Ensure: export PATH=\"$HOME/.local/bin:$PATH\"" >&2
	fi
	echo "  - Or set: NU_BOTS_UV=/path/to/uv" >&2
	exit 127
fi

exec "${uv_bin}" run --script "${script_dir}/nuclear/b.py" "$@"
