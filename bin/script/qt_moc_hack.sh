#!/usr/bin/env bash

# moc_hack.sh
# 
#
# Created by Jim Mainprice on 02/11/09.
# Copyright 2009 __MyCompanyName__. All rights reserved.

#!/usr/bin/env bash

MOC="$1"
shift
FILE="$1"
shift
CPPFLAGS="$@"
DIR="$(dirname "${FILE}")"
#echo $UIC
#echo $FILE
#echo $DIR

echo $CPPFLAGS

# Test that the file is there
[[ -f "${FILE}" ]]
NotFileExist=$?
if [[ $NotFileExist == 1 ]]; then
echo "file "${FILE}" doesnt exist etc crash"
exit 1
fi

# Test there is a moc included
mocFileBasename="$(grep '#include \"moc_.*' "${FILE}" | sed -e 's/#include "moc_\(.*\).cpp"/\1/')"
if [[ -z "${mocFileBasename}" ]]; then
echo "no include found"
exit 0
fi

# Runs moc on the header file
#echo "${mocFileBasename}".hpp
file=${FILE##*/}
base=${file%%.*}
echo "Runing MOC on header ${DIR}/${base}".hpp
$MOC "${DIR}/${base}".hpp ${CPPFLAGS} -o moc_"${base}".cpp
