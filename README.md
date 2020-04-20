![C/C++ CI](https://github.com/malikkirchner/froggy/workflows/C/C++%20CI/badge.svg)

# Froggy

## Build and test

Install Conan according to the instructions at [docs.conan.io](https://docs.conan.io/en/latest/installation.html).
From the repository root run following commands to build and test.
```bash
# make build directory
mkdir build
cd build
# configure build system
cmake .. -DCMAKE_BUILD_TYPE=Debug|Release|...
# build
cmake --build .
# test
cmake --build . --target test
```

## Usage

Take a look at `test/unit_test.cpp` for an example.

## Example

A moon shot could look like this. The Earth is blue, the Rocket is orange and the Moon is gray. The lines show the body's trajectories.
![moon shot](/moon_shot.png)
