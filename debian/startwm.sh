#!/bin/sh

if test -r /etc/default/locale; then
	. /etc/default/locale
	export LANG LANGUAGE
fi

. /etc/X11/Xsession
