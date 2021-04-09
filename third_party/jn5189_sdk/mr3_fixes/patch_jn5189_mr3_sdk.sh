#!/bin/bash

CURR_PATH="`dirname \"$0\"`" 

if [[ ! -d $NXP_JN5189_SDK_ROOT ]]; then
    echo "NXP_JN5189_SDK_ROOT is not set"
    exit 1
fi

cp $CURR_PATH/board_utility.c  "$NXP_JN5189_SDK_ROOT"/boards/jn5189dk6/wireless_examples/openthread/enablement
cp $CURR_PATH/board_utility.h  "$NXP_JN5189_SDK_ROOT"/boards/jn5189dk6/wireless_examples/openthread/enablement
cp $CURR_PATH/startup_jn5189.c "$NXP_JN5189_SDK_ROOT"/devices/JN5189/mcuxpresso

echo "JN5189 SDK Master Release 3 was patched!"
exit 0
