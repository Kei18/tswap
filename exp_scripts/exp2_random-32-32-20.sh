#!/bin/sh
map="random-32-32-20.map"
agents_list="30 70 110"
flocking_blocks=0
scen_start=1
scen_end=50
force=0

set -e

solver="TSWAP"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="FlowNetwork -l"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force
