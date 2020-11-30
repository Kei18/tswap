#!/bin/sh

## args
map=$1
agents_list=$2
solver=$3
scen_start=$4
scen_end=$5
flocking_blocks=$6

PROJECT_DIR=`dirname $0`/..
map_trimed=${map%.map}

## check git status
GIT_STATUS_RESULT=`git status -s`
if [ ${#GIT_STATUS_RESULT} -ne 0 ]
then
    echo "Untracked changes exist. Commit them beforehand."
    git status -s
    exit 1
fi
GIT_RECENT_COMMIT=`git log -1 --pretty=format:"%H"`

## create output directory
EXP_DATE=`date +%Y-%m-%d-%H-%M-%S`
OUTPUT_DIR=$PROJECT_DIR/../data/$EXP_DATE/
mkdir -p $OUTPUT_DIR

## build
rm -rf $PROJECT_DIR/build
mkdir $PROJECT_DIR/build
cd $PROJECT_DIR/build
cmake ..
make

## main
for agent_num in ${agents_list[@]}
do
    scen=$scen_start
    while [ $scen -lt $scen_end ]
    do
        scen_file="${map_trimed}_${agent_num}agents_${flocking_blocks}blocks_${scen}.txt"
        if [ $flocking_blocks -eq 0 ]
        then
            scen_file="${map_trimed}_${agent_num}agents_${scen}.txt"
        fi
        echo $scen_file

        $PROJECT_DIR/build/app \
                    -i $PROJECT_DIR/instances/$scen_file \
                    -o $OUTPUT_DIR/$scen_file \
                    -s $solver
        scen=`expr $scen + 1`
    done
done

## create status file
STATUS_FILE=$OUTPUT_DIR/status.txt
{
    echo start:$EXP_DATE
    echo end:`date +%Y-%m-%d-%H-%M-%S`
    echo used-commit:$GIT_RECENT_COMMIT
    echo map:$map
    echo agents:$agents_list
    echo solver:$solver
    echo scen_start:$scen_start
    echo scen_end:$scen_end
    echo flocking_blocks:$flocking_blocks
} > $STATUS_FILE
