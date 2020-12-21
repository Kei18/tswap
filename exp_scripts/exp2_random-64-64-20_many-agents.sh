#!/bin/sh
map="random-64-64-20.map"
agents_list="500 1000 1500 2000"
flocking_blocks=0
scen_start=1
scen_end=50

set -e

solver="TSWAP"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks

solver="FlowNetwork -l"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks
