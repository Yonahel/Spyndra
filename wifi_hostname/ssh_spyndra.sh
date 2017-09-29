#!/bin/bash


#for i in "$@"
#do 
#  case $1 in
#    -i=*) 
#    IP="${i#*=}"
#    shift;;
#    -k=*) 
#    KEY=${i#*=}
#    shift;;
#  esac
#done

#echo Connecting to PI on Spyndra...
#echo IP = $IP
#echo KEY = $KEY

#echo Connecting to pi@"$1"...
echo Connecting to pi@spyndra.local...
ssh_key='spyn_prk.txt'
if [ -f $ssh_key ]; then
  chmod 600 $ssh_key  # limit access permission 
  ssh -i $ssh_key "pi@spyndra.local"

else
  echo File $ssh_key does not exist.
fi
