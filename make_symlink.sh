#!/bin/bash

make_symlink() {

  ros_ws="$HOME/catkin_ws/src/"

  if [ -d "$ros_ws" ]; then

    if [ -d "$ros_ws"/"$2" ]; then
        echo "The folder $2 already exist."
        return
    fi

    ln -s "$1" "$ros_ws"
    echo "Successfully made symlink for $2."

  else
    echo "ERROR: The work space directory $ros_ws does not exist."
    exit 1
  fi
}

## Get current folder and make sure it is *RoTools*
CURR_FOLDER=${PWD##*/}
if [ "$CURR_FOLDER" != "RoTools" ]; then
  echo "ERROR: you need to run the script from the RoTools directory."
  echo "$CURR_FOLDER"
  exit 1
fi

echo "Making symlink for $CURR_FOLDER";
make_symlink "$PWD" "$CURR_FOLDER"

exit 0