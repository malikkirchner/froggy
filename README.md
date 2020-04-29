![C/C++ CI](https://github.com/malikkirchner/froggy/workflows/C/C++%20CI/badge.svg)

# Froggy

This repository contains the example code to ["Fly Me To The Moon"](https://medium.com/@malik.kirchner/fly-me-to-the-moon-195f853dd708).

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
[2020-04-29 15:53:22.243] [info] Starting simulation ...
[2020-04-29 15:53:29.437] [info] Rendering orbits ...
[2020-04-29 15:53:29.437] [info] Please find the diagram at /home/.../froggy/build/moon_shot.png
[2020-04-29 15:53:29.706] [info] Flight stats:
Iterations               : 64026513 (8580342.133/s)
Simulation duration      : 7.462s
Flight duration          : 640265.13s (7.41d)
Lowest velocity          : 336.75m/s (1212.29km/h)
Hightest velocity        : 11114.93m/s (40013.76km/h)
Lowest acceleration      : 0.00m/s^2 (0.00g)
Hightest acceleration    : 33.20m/s^2 (3.38g)
Rocket trajectory length : 798481.650km
Earth trajectory length  : 7970.830km
Moon trajectory length   : 647870.251km
```

This picture shows take off and landing of the Rocket.
![moon shot](/moon_shot_zoom_earth.png)

This picture shows the Moon at its closest to the Rocket's trajectory.
![moon shot](/moon_shot_zoom_moon.png)