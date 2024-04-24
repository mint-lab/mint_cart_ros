#!/bin/bash

# Set file name and path
UDEV_RULES="88-mint-cart.rules"
SCRIPT_DIR=$(dirname "$(readlink -f "$0")") # Absolute path to this script file
SOURCE_PATH="$SCRIPT_DIR/$UDEV_RULES"
TARGET_PATH="/etc/udev/rules.d/$UDEV_RULES"

# Check root permission
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run with root privileges. Please use with 'sudo'"
    exit 1
fi

# Copy udev rules file to the target path
cp -v "$SOURCE_PATH" "$TARGET_PATH"

# Apply changes to udev management service
sudo service udev reload
sudo service udev restart

echo "udev rules is updated!"
echo "After unplugging and re-plugging the device USB, enter the command below to check the changes"
echo "\"ls -al /dev/tty*\""
