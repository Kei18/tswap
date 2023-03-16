#!/bin/sh
map="random512-20-0.map"
agents_list="30 70 110"
flocking_blocks=0
scen_start=1
scen_end=50
force=1

set -e

solver="TSWAP -m 2"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="TSWAP -m 5"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="FlowNetwork -l"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force
