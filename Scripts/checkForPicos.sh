#!/bin/bash
#
#-------------------------------------------------------------------------------------------------------------
# "checkForPicos.sh" is a helper script to check for PICO terminal IO files and automatically opens a terminal
# window for a new PICO. The script could run in the background and save some typing for connecting the PICOs.
# On the Mac the PICO opens a terminal file under the path "/dev/cu.usbmodem" and the USB port number.
#
#-------------------------------------------------------------------------------------------------------------
known_devices=()

#-------------------------------------------------------------------------------------------------------------
# Check for currently connected PICOs. We iterate through the device files found and if it is a new file,
# open the terminal window and add it to the liust of known terminals.
#
#-------------------------------------------------------------------------------------------------------------
function check_devices {
   
    current_devices=($(ls /dev/cu.usbmodem* 2>/dev/null))

    if [[ ${#current_devices[@]} -eq 0 ]]; then
        echo "No devices found."
        return
    fi

    echo "Devices found:"
    echo "${current_devices[@]}"

    for device in "${current_devices[@]}"; do
        # Check if the device is not in known_devices
        if [[ ! " ${known_devices[*]} " =~ " ${device} " ]]; then
            known_devices+=("$device")
            echo "New device detected: $device"
            osascript -e "tell application \"Terminal\" to do script \"/opt/homebrew/bin/minicom -D $device -b 115200\""
        fi
    done

}

#-------------------------------------------------------------------------------------------------------------
# Main loop.
#
#-------------------------------------------------------------------------------------------------------------
while true; do
    check_devices
    sleep 2  # Check every 2 seconds
done

#-------------------------------------------------------------------------------------------------------------
#
# Notes:
#
# make it an executable:    chmod +x checkForPicos.sh
#
# run in background:        nohup ./chckForPicos.sh
#
# kill it:                  ps aux | grep checkForPicos.sh
#                           kill <PID>
#
#-------------------------------------------------------------------------------------------------------------