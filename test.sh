#!/bin/bash

userinput=""
echo "Press \"q\" key to quit"

while read -r -n1 key
do

    if [[ $key == q ]] ; then
        break;
    fi

done
printf "\nsee you\n"
