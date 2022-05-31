#!/bin/sh
set -o errexit

# Rebuild viam cartographer.
pushd build
ninja
popd
