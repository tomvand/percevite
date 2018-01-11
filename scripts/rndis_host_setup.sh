#! /bin/sh

CURRENT_TTY=$(tty)
if [ $? -ne 0 -o "$CURRENT_TTY" = "" ]; then
    CURRENT_TTY=/dev/console
fi
echo " => $0 ($@) : CURRENT_TTY = $CURRENT_TTY" | ulogger

my_print ()
{
    echo "$0 : $@" | tee $CURRENT_TTY | ulogger
}

if [ $# -eq 0 ]; then
    my_print "No command... => stops (Possible command are: create_net_interface , remove_net_interface )"
    exit 1
else
    RNDIS_HOST_SCRIPT_CMD=$1
fi

case $RNDIS_HOST_SCRIPT_CMD in
    create_net_interface)
        # turn off wifi interface, as Android device use the same subnet
        # ifconfig eth0 down # Disabled 2018-03-07 Tom v Dijk

        IFACE_NAME=usb
        my_print "Create Interface :"
        
        # Changed to static IP for SLAMDunk 2018-03-07 Tom v Dijk
        # ifconfig $IFACE_NAME up
        # udhcpc -i $IFACE_NAME
        ifconfig $IFACE_NAME 192.168.45.28 up
        
        # Set up IP forwarding 2018-03-07 Tom v Dijk
        echo 1 > /proc/sys/net/ipv4/ip_forward
        
        my_print "$IFACE_NAME is up Using address $IFACE_IP_AP"
        my_print "USB_ETH_OK"

        exit 0
        ;;
    remove_net_interface)
        # restore wifi interface
        ifconfig eth0 up
        exit 0
        ;;
    *)
        my_print "Unknown Command..."
        exit 1
        ;;
esac

