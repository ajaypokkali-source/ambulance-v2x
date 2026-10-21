#!/usr/bin/env bash
# Runs baseline / signals-only / full V2X and writes results/comparison.png
cd "$(dirname "$0")"
exec .venv/bin/python compare.py "$@"
