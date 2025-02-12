#!/bin/bash

# Usage: ./build.sh for normal build, ./build.sh -c for clean build (remove current build directory)

if [ "$CONDA_DEFAULT_ENV" != "rpc" ]; then
  echo "Activating rpc environment"
  eval "$(conda shell.bash hook)"
  conda activate rpc
fi

while getopts "c" flag; do
  case $flag in
    c)
      echo "Removing build directory..."
      rm -rf build
      ;;
    \?)
      echo "Invalid flag"
      exit 1
      ;;
  esac
done

echo "Creating build directory..."
mkdir -p build
cd build

echo "Running cmake..."
cmake ..
if [ $? -ne 0 ]; then
  echo "cmake was unsuccessful. Exiting..."
  exit 1
fi

echo "Building..."
make -j4

if [ $? -eq 0 ]; then
  echo "Build succeeded."
else
  echo "Build failed."
  exit 1
fi
