#!/bin/bash

# reloads the BT driver on OSX mountain lion
# run after unloading the drivers using the uload script
# make sure you're done using BT on the VM host
# found here:
# https://www.virtualbox.org/ticket/2372

sudo launchctl load /System/Library/LaunchDaemons/com.apple.blued.plist
sudo kextload -b com.apple.iokit.IOBluetoothSerialManager
sudo kextload -b com.apple.iokit.BroadcomBluetoothHCIControllerUSBTransport