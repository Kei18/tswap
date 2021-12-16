#!/bin/sh
map="brc202d.map"
agents_list="100 500 1000 2000"

flocking_blocks=0
scen_start=1
scen_end=50
force=0

set -e

# bottleneck-linear
solver="TSWAP -m 0"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

# bottleneck-linear without lazy evaluation
solver="TSWAP -m 1"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

# bottleneck
solver="TSWAP -m 2"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

# linear
solver="TSWAP -m 3"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

# greedy
solver="TSWAP -m 4"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

# greedy with refinement
solver="TSWAP -m 5"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

# greedy with refinement without lazy evaluation
solver="TSWAP -m 6"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

# greedy with refinement for sum-of-costs
solver="TSWAP -m 7"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force
