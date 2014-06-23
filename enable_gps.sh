killall gpsd 2> /dev/null
gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock

