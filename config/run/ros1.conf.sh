#!/usr/bin/env bash

STEPIT_ARGS="${STEPIT_ARGS:-} -P ros"
STEPIT_PLUGIN_ARGS="${STEPIT_PLUGIN_ARGS:-} -r __ns:=${NS:-/stepit}"
