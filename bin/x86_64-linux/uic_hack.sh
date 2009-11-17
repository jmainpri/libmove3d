#!/bin/sh

# uic_hack.sh
# 
#
# Created by Jim Mainprice on 02/11/09.
# Copyright 2009 __MyCompanyName__. All rights reserved.

UIC="$1"
FILE="$2"
DIR="$(dirname "${FILE}")"
#echo $UIC
#echo $FILE
#echo $DIR

# Test that the file is there
[[ -f "${FILE}" ]]
ret=$?
if [[ $ret == 1 ]]; then
echo "file "${FILE}" doesnt exist etc crash"
exit 1
fi

# Test there is a *.ui file included
uiFileBasename="$(grep '#include \"ui_.*' "${FILE}" | sed -e 's/#include "ui_\(.*\).hpp"/\1/')"
if [[ -z "${uiFileBasename}" ]]; then
echo "no include found"
exit 0
fi

# Runs uic on the header file
$UIC "${DIR}/${uiFileBasename}".ui > "${DIR}/"ui_"${uiFileBasename}".hpp