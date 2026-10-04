#!/usr/bin/env bash
# Usage: ./run.sh [v2x_city.py options]   e.g.  ./run.sh --track amb1
cd "$(dirname "$0")"
exec .venv/bin/python v2x_city.py "$@"
