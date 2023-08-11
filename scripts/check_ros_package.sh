#!/bin/bash

containsElement () {
  local e match="$1"
  shift
  for e; do [[ "$e" == "$match" ]] && return 1; done
  return 0
}

if [ "$#" -ne 1 ]; then
    echo "Skipping required package check, no list of packages provided!"
    exit 0
fi

if [ ! -f "$1" ]; then
    echo "Skipping required package check, provided file does not exits!"
    exit 0
fi

result=0
RED='\e[31m'
GREEN='\e[32m'
NC='\e[39m' # No Color
IFS=' - '
declare meta_array=()

while read line; do

  read -ra str_arr <<< "$line"

  VAR=$(rospack list-names | grep ${str_arr[0]})
  echo -e "checking: ${str_arr[0]} ... \c"
  if [ -z "${VAR}" ]; then
    if [[ "${#str_arr[@]}" -eq 2 ]]; then
      echo -e "${RED}missing${NC} (part of metapackage ${str_arr[1]})"
        containsElement "${str_arr[1]}" "${meta_array[@]}"
        if [[ "$?" -eq 0 ]]; then
          meta_array+=("${str_arr[1]}")
        fi
    else
      echo -e "${RED}missing${NC}"
    fi

    result=1
  else
    echo -e "${GREEN}found${NC}"
  fi
done < $1


if [[ $result -ne 0 ]]; then

  echo -e "\n${RED}Some required packages were not found, the simulation will probably not work correctly.${NC}"

  if [[ "${#meta_array[@]}" -ne 0 ]]; then
    echo -e "You are missing the following metapackages:"
    echo "${meta_array[*]}"
  fi
  echo -e ""

  default=n
  { read -t 10 -n 2 -p $'Proceed to run the simulation? [y/n] (default: '"$default"$')' resp || resp=$default ; }
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then
    exit 0
  else
    exit 1
  fi
fi

exit 0
