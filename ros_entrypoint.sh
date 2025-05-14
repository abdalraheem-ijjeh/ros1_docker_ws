#!/usr/bin/env bash
set -e
# (The Dockerfile ENTRYPOINT does the sourcing & run)
exec "$@"

