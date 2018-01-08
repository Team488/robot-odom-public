#!/bin/bash 

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue

        if [ "Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001" == "$ID_SERIAL" ]; then 
            chmod 666 "/dev/$devname"
            echo "/dev/$devname - $ID_SERIAL modified"
        fi
    )
done;