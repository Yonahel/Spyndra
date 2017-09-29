#!/bin/bash
echo $1
cmd="scp -i spyn_prk.txt pi@spyndra.local:$1 $2"
echo $cmd
$cmd
#for i in "$@"
#do
#  case $i in
#    -ip=*)
#      IP="${ip#*=}"
#      shift;;
#    -from=*)
#      SOURCE="$from#*=}"
#      shift;;
#  esac
#done

