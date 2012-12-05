#!/bin/bash

# unloads the BT driver from OSX mountain lion
# run before trying to use BT in host on VM
# found here:
# https://www.virtualbox.org/ticket/2372

sudo launchctl unload /System/Library/LaunchDaemons/com.apple.blued.plist
sudo kextunload -b com.apple.iokit.IOBluetoothSerialManager
sudo kextunload -b com.apple.iokit.BroadcomBluetoothHCIControllerUSBTransport 