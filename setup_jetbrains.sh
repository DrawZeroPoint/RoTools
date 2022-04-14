#!/bin/bash

APP_PATH=$HOME/.local/share/JetBrains/Toolbox/apps

select_highest_version() {

  # The regex starts with ^ and end with $. In the middle, [0-9]{3} is the major version with 3 numbers,
  # \. is the separator between version numbers. The last minor number could have 2 or 3 digits.
  kREGEX_VERSION='^[0-9]{3}\.[0-9]{4}\.[0-9]{2,3}$'

  MAJOR_VERSION=0
  MIDDLE_VERSION=0
  MINOR_VERSION=0
  SELECTED_VERSION=0.0.0

  for dir in $APP_PATH/$1/ch-0/*.*.*; do
    if [[ $(basename "$dir") =~ $kREGEX_VERSION ]]; then
      echo "Found $1 version: $(basename "$dir")"

      LOCAL_MAJOR=$(echo $(basename "$dir") | cut -d'.' -f 1)
      LOCAL_MIDDLE=$(echo $(basename "$dir") | cut -d'.' -f 2)
      LOCAL_MINOR=$(echo $(basename "$dir") | cut -d'.' -f 3)

      if [[ $LOCAL_MAJOR -gt $MAJOR_VERSION ]]; then
        SELECTED_VERSION=$LOCAL_MAJOR.$LOCAL_MIDDLE.$LOCAL_MINOR
      fi
    fi
  done

  if [[ ":$PATH:" == *":$APP_PATH/$1/ch-0/$SELECTED_VERSION/bin:"* ]]; then
    echo "$1's bin folder has already been added to PATH"
  else
    echo "export PATH=\$JB_PATH/$1/ch-0/$SELECTED_VERSION/bin:\$PATH" >>~/.bashrc
    echo "Added the bin folder of $1 $SELECTED_VERSION to PATH"
  fi
}

if [ -d "$APP_PATH" ]; then
  echo "Found JetBrains Toolbox"
else
  echo "JetBrains Toolbox not found in the default location:
  $APP_PATH,
  you may need to install it and rerun this script."
  exit 0
fi

if [ -z ${JB_PATH+x} ]; then
  echo "export JB_PATH=$APP_PATH" >>~/.bashrc
  echo "Added JB_PATH $APP_PATH to bashrc"
else
  echo "JB_PATH has been set to '$JB_PATH'"
fi

select_highest_version "CLion"
select_highest_version "PyCharm-P"

exit 0
