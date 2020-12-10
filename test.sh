#!/bin/bash

gnome-terminal --tab -e 'sh -c "rosrun soa_pkg navigation; exec bash"'s


userinput=""
echo "Press \"q\" key to quit"

while read -r -n1 key
do

    if [[ $key == q ]] ; then
        break;
    fi

done
printf "\nsee you\n"
