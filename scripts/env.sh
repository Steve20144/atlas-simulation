#!/usr/bin/env bash
# Source this in Git Bash on Windows before running make, uv, node or npm:
#   source scripts/env.sh
# On Linux and macOS it is harmless (only prepends directories that exist).
_tl_add() { [ -d "$1" ] && case ":$PATH:" in *":$1:"*) ;; *) PATH="$1:$PATH";; esac; }
_tl_add "$HOME/.local/bin"
if command -v cygpath >/dev/null 2>&1 && [ -n "$LOCALAPPDATA" ]; then
  _tl_add "$(cygpath -u "$LOCALAPPDATA")/Programs/nodejs"
fi
export PATH
unset -f _tl_add
