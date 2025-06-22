#!/bin/bash
# 
# LoadPico - a shell script to reboot a PICO and load a new imgage
#
# The script will first build the USB address and try to locate the PICO. We will also derive the
# port number from the address. Next, the PICO is loaded with a new image.
#
if [ "$#" -lt 2 ]; 
then
	echo "Usage: loadpico <filename> <usb address>"
    echo "The file name is the UF2 file to load"
    echo "The USB address is the subset needed for the device file. (e.g. /dev/cu.usbmodem[XXXX]1 )"
	exit
fi

# Function to get the port number given the USB address in substring pattern form. We will get the USB 
# info from the system profiler descriptors, grep for USB data for attached PICOs and then analyze the
# Location ID attribute. When the US address matches, we return the port number.
#
get_port_num_by_usb_address() {

    local usb_info=$(system_profiler SPUSBDataType | grep -A 10 "Pico:" | grep "Location ID")

    # Loop through each line of the usb_info
    echo "$usb_info" | while read -r line; do

        if [[ "$line" == *"Location ID: "* ]]; then

            if [[ "$line" == *"$1"* ]]; then

		        current_usb_address=$(echo "$line" | awk '{print $3}')
                port_number=$(echo "$line" | awk '{print $5}')
                echo "$port_number"
                return    
            fi
        fi
	done
}

# Function to extract a portion of the USB address. This part is needed for the /dev/... file
# descriptor that is the USB stdio file opened by the PICO. 
#
extract_usb_address_chars() {

	extracted_chars=$(echo "$1" | cut -c 4-6)
	echo "$extracted_chars"
}

# Function to build a device file, based on the address. We concatenate the prefix with the function
# parameter and append a "1", which seems to be the first stdio port for the pico.
#
build_dev_file_name() {

    local prefix="/dev/cu.usbmodem"
    local tmp="$prefix$1"
    echo "$prefix""$1""1"
}


# Load the PICO with a new image
#
port_number=$(get_port_num_by_usb_address "$1")

if [[ -n "$port_number" ]]; then

    echo "Loading the PICO at $1 with $2"
    
    # we need to use the picotool load command
    picotool load -v $2 --address $port_number -f
    picotool reboot --address $port_number -f

    sleep 1

    # echo "Start Terminal for Device File Name: $(build_dev_file_name "$1" )"
    # screen $(build_dev_file_name "$1" ) 115200
    
    echo "loaded"
   
else
    echo "PICO not found"
fi
