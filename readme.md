Unlabeled Multi-Agent Path Finding
===
![test](https://github.com/Kei18/unlabeled-MAPF/workflows/test/badge.svg?branch=dev)
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE.txt)

A simulator and visualizer of Unlabeled/Anonymous Multi-Agent Path Finding (MAPF).
It is written in C++(17) with [CMake](https://cmake.org/) build and tested on OSX 10.15.
The repository uses [Google Test](https://github.com/google/googletest).
The visualizer uses [openFrameworks](https://openframeworks.cc).

## Demo

## Building

```
git clone https://github.com/Kei18/mapf-IR
cd mapf
mkdir build
cd build
cmake ..
make
```

## Usage
PIBT
```sh
./app -i ../instances/sample.txt -s PIBT -o result.txt -v
```

IR (the result is saved in `result.txt`)
```sh
./app -i ../instances/random-32-32-20_70agents_1.txt -s IR -n 50 -t 3000 -v
```

You can find details and explanations for all parameters with:
```sh
./app --help
```

Please see `instances/sample.txt` for parameters of instances, e.g., filed, number of agents, time limit, etc.

### Output File

This is an example output of `../instances/sample.txt`.
Note that `(x, y)` denotes location.
`(0, 0)` is the left-top point.
`(x, 0)` is the location at `x`-th column and 1st row.
```
instance= ../instances/sample.txt
agents=100
map_file=arena.map
solver=PIBT
solved=1
soc=3403
makespan=68
comp_time=58
starts=(32,21),(40,4),(20,22),(26,18), [...]
goals=(10,16),(30,21),(11,42),(44,6), [...]
solution=
0:(32,21),(40,4),(20,22),(26,18), [...]
1:(31,21),(40,5),(20,23),(27,18), [...]
[...]
```

## Visualizer

### Building
It takes around 10 minutes.
```sh
git submodule init
git submodule update
sh ./openFrameworks/scripts/osx/download_libs.sh
cd visualizer/
make build
cd ..
chmod +x ./visualize.sh
```

### Usage
```sh
cd build
../visualize.sh result.txt
```

You can manipulate it via your keyboard. See printed info.

## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Notes
- Maps in `maps/` are from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).
  When you add a new map, please place it in the `maps/` directory.
- The font in `visualizer/bin/data` is from [Google Fonts](https://fonts.google.com/).
- Scripts for the experiments are in `exp_scripts/`.

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. candidate at Tokyo Institute of Technology, working on multiple moving agents.

## Reference
