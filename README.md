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

Some flight stats are printed to std::out.
```
[2020-04-23 20:01:11.645] [info] Starting simulation ...
[2020-04-23 20:01:19.463] [info] Rendering orbits ...
[2020-04-23 20:01:19.463] [info] Please find the diagram at /home/.../froggy/build/moon_shot.png
[2020-04-23 20:01:19.495] [info] Flight stats:
Iterations               : 64334701 (8195503.312/s)
Simulation duration      : 7.850s
Flight duration          : 643347.01s (7.45d)
Lowest velocity          : 336.46m/s (1211.26km/h)
Hightest velocity        : 11111.18m/s (40000.25km/h)
Lowest acceleration      : 0.00m/s^2 (0.00g)
Hightest acceleration    : 33.20m/s^2 (3.38g)
Rocket trajectory length : 794408.553km
Earth trajectory length  : 8009.273km
Moon trajectory length   : 650994.930km
```

This picture shows take off and landing of the Rocket.
![moon shot](/moon_shot_zoom_earth.png)

This picture shows the Moon at its closest to the Rocket's trajectory.
![moon shot](/moon_shot_zoom_moon.png)