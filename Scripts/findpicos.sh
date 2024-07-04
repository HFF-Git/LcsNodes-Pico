#!/bin/bash
# A little script to get PICO location ID

# Get the USB information
usb_info=$(system_profiler SPUSBDataType | grep -A 10 "Pico:" | grep "Location ID")

# Iterate through each line of the usb_info

echo "$usb_info" | while read -r line; do
    
    # Extract USB address and port number using awk
    usb_address=$(echo "$line" | awk '{print $3}')
    port_number=$(echo "$line" | awk '{print $5}')
    
    # Print the values
    echo "USB Address: $usb_address"
    echo "Port Number: $port_number"

done

