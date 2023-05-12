#!/bin/sh
flocking_blocks=0
scen_start=1
scen_end=50
force=0

set -e

map="random-64-64-20.map"
agents_list="110 500 1000 2000"

solver="NaiveTSWAP -m 0"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="TSWAP -m 0"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force


map="lak303d.map"
agents_list="100 500 1000 2000"

solver="NaiveTSWAP -m 0"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="TSWAP -m 0"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force


map="random-64-64-20.map"
agents_list="110 500 1000 2000"

solver="NaiveTSWAP -m 5"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="TSWAP -m 5"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force


map="lak303d.map"
agents_list="100 500 1000 2000"

solver="NaiveTSWAP -m 5"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force

solver="TSWAP -m 5"
sh `dirname $0`/run.sh $map "$agents_list" "$solver" $scen_start $scen_end $flocking_blocks $force
