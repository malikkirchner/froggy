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

```
[2020-04-23 13:46:22.155] [info] Starting simulation ...
[2020-04-23 13:46:34.123] [info] width x height : 1304 x 1080
[2020-04-23 13:46:34.123] [info] origin         : 288, 77
[2020-04-23 13:46:34.123] [info] Rendering orbits ...
[2020-04-23 13:46:34.123] [info] Please find the diagram at /.../moon_shot.png
[2020-04-23 13:46:34.156] [info] Flight stats:
Iterations               : 67827640
Simulation duration      : 12.001s
Flight duration          : 678276.40s (7.85d)
Lowest velocity          : 252.17m/s (907.83km/h)
Hightest velocity        : 11084.68m/s (39904.83km/h)
Lowest acceleration      : 0.00m/s^2 (0.00g)
Hightest velocity        : 98.81m/s^2 (10.07g)
Rocket trajectory length : 785855.554km
Earth trajectory length  : 8445.041km
Moon trajectory length   : 686414.233km
```
