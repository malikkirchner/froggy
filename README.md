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
cmake .. -DCMAKE_BUILD_TYPE=Debug|Release|... [ -DUSE_GTK=ON ]
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
[2020-04-23 19:31:39.085] [info] Starting simulation ...
[2020-04-23 19:31:48.178] [info] width x height : 1506px x 1080px
[2020-04-23 19:31:48.178] [info] origin         : 481px, 77px
[2020-04-23 19:31:48.178] [info] Rendering orbits ...
[2020-04-23 19:31:48.178] [info] Please find the diagram at /home/.../froggy/build/moon_shot.png
[2020-04-23 19:31:48.217] [info] Flight stats:
Iterations               : 75489569 (8267393.385/s)
Simulation duration      : 9.131s
Flight duration          : 754895.69s (8.74d)
Lowest velocity          : 138.41m/s (498.28km/h)
Hightest velocity        : 11085.95m/s (39909.43km/h)
Lowest acceleration      : 0.00m/s^2 (0.00g)
Hightest acceleration    : 33.21m/s^2 (3.39g)
Rocket trajectory length : 795888.745km
Earth trajectory length  : 9401.295km
Moon trajectory length   : 764138.733km
```
