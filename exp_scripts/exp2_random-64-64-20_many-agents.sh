#!/bin/sh
map="random-64-64-20.map"
agents_list="500 1000 1500 2000"
flocking_blocks=0
scen_start=1
scen_end=50
force=0

set -e

solver="TSWAP -m 2"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="TSWAP -m 5"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="FlowNetwork -d"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force
