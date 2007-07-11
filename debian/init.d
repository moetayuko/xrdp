#! /bin/sh
#
# start/stop xrdp and sesman daemons

### BEGIN INIT INFO
# Provides:          xrdp
# Required-Start:    $network
# Required-Stop:     $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start xrdp daemon
### END INIT INFO

PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin
DAEMON=/usr/bin/xrdp
NAME=xrdp
DESC=xrdp

test -x $DAEMON || exit 0

if [ -r /etc/default/$NAME ]; then
   . /etc/default/$NAME
fi

set -e

case "$1" in
  start)
	echo -n "Starting $DESC: "
        start-stop-daemon --start --quiet --oknodo --pidfile /var/run/$NAME.pid \
	    --exec $DAEMON
	echo -n "$NAME"
	[ "$SESMAN_START" = "yes" ] && { \
            start-stop-daemon --start --quiet --oknodo --pidfile /var/run/sesman.pid \
	       --exec /usr/bin/sesman
	    echo -n " sesman"
	}
	echo "."
	;;
  stop)
	echo -n "Stopping $DESC: "
        start-stop-daemon --stop --quiet --oknodo --pidfile /var/run/sesman.pid \
	    --exec /usr/bin/sesman
	echo -n "sesman "
	start-stop-daemon --stop --quiet --oknodo --pidfile /var/run/$NAME.pid \
		--exec $DAEMON
	echo "$NAME."
	;;
  restart)
	$0 stop
	sleep 1
	$0 start
	;;
  *)
	N=/etc/init.d/$NAME
	# echo "Usage: $N {start|stop|restart|reload|force-reload}" >&2
	echo "Usage: $N {start|stop|restart|force-reload}" >&2
	exit 1
	;;
esac

exit 0
