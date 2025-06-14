#!/bin/bash
# Wrapper script to run commands with depot_tools in PATH, filtering out problematic Windows paths

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPOT_TOOLS_PATH="$SCRIPT_DIR/depot_tools"

# Filter out Windows paths from PATH to avoid shell escaping issues in WSL
# Keep only Linux paths (those starting with /)
CLEAN_PATH=""
IFS=':'
for path_component in $PATH; do
    if [[ "$path_component" == /* ]]; then
        if [ -z "$CLEAN_PATH" ]; then
            CLEAN_PATH="$path_component"
        else
            CLEAN_PATH="$CLEAN_PATH:$path_component"
        fi
    fi
done

# Add depot_tools to the front of the clean PATH
export PATH="$DEPOT_TOOLS_PATH:$CLEAN_PATH"

# Execute the command passed as arguments
exec "$@"
