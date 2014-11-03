# $NetBSD$

KMOD=	vboxtime
IOCONF=	vboxtime.ioconf
SRCS=	vboxtime.c

WARNS=	5

_BSD_IOCONF_MK_USER_=1
.include <bsd.ioconf.mk>
.include <bsd.own.mk>
.include <bsd.kmodule.mk>
