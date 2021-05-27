set -e
force=0
sh `dirname $0`/run.sh random-64-64-20.map "50" "FlowNetwork -t 28" 1 1 0 $force
sh `dirname $0`/run.sh random-64-64-20.map "110" "FlowNetwork -t 18" 1 1 0 $force
