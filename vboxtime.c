/* $NetBSD$ */

/*
 * Copyright (c) 2014 KAMADA Ken'ichi.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <uvm/uvm_extern.h>
#include <dev/pci/pcidevs.h>
#include <dev/pci/pcivar.h>

#include "vboxtime.h"

#define VBOXTIME_SYNC_INTERVAL		1	/* in seconds */

#define VMMDEV_BAR0		(PCI_MAPREG_START + 0)	/* I/O */
#define VMMDEV_BAR1		(PCI_MAPREG_START + 4)	/* Memory */

struct vboxtime_softc {
	/* Generic device information. */
	device_t		sc_dev;
	/* I/O space for VirtualBox VMM device. */
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;
	bus_size_t		sc_iosize;
	/* Memory space for VirtualBox VMM device. */
	bus_space_tag_t		sc_memt;
	bus_space_handle_t	sc_memh;
	bus_size_t		sc_memsize;

	struct callout		sc_sync_callout;
	int			sc_sync_ticks;
};

static int vboxtime_match(device_t, cfdata_t, void *);
static void vboxtime_attach(device_t, device_t, void *);
static int vboxtime_detach(device_t, int);
static int vboxtime_modcmd(modcmd_t, void *);

CFATTACH_DECL_NEW(vboxtime, sizeof(struct vboxtime_softc),
    vboxtime_match, vboxtime_attach, vboxtime_detach, NULL);

MODULE(MODULE_CLASS_DRIVER, vboxtime, "pci");

#ifdef _MODULE
#include "ioconf.c"
#endif

static int vboxtime_init(struct vboxtime_softc *);
static void vboxtime_sync(void *);
static void vboxtime_make_req_header(VMMDevRequestHeader *,
    VMMDevRequestType, uint32_t);

static int
vboxtime_match(device_t parent, cfdata_t cf, void *aux)
{
	struct pci_attach_args *pa = aux;

	if (PCI_VENDOR(pa->pa_id) == PCI_VENDOR_VIRTUALBOX &&
	    PCI_PRODUCT(pa->pa_id) == PCI_PRODUCT_VIRTUALBOX_GUEST)
		return 1;
	return 0;
}

static void
vboxtime_attach(device_t parent, device_t self, void *aux)
{
	struct vboxtime_softc *sc = device_private(self);
	struct pci_attach_args *pa = aux;
	pcireg_t rev;

	sc->sc_dev = self;
	rev = PCI_REVISION(pa->pa_class);

	/*
	 * Do this by myself.  If we call pci_aprint_devinfo(pa, NULL),
	 * the kprintf family malfunctions.  It autoloads the "pciverbose"
	 * module, and it seems to me that recursive loading of modules
	 * has something to do with this.
	 */
	aprint_normal(": VirtualBox Guest Service (rev. 0x%02x)\n", rev);
	aprint_naive("\n");

	callout_init(&sc->sc_sync_callout, 0);

	if (rev != 0) {
		aprint_error_dev(sc->sc_dev, "unknown revision\n");
		return;
	}

	if (pci_mapreg_map(pa, VMMDEV_BAR0, PCI_MAPREG_TYPE_IO, 0,
	    &sc->sc_iot, &sc->sc_ioh, NULL, &sc->sc_iosize)) {
		aprint_error_dev(sc->sc_dev, "can't map i/o space\n");
		return;
	}
	if (pci_mapreg_map(pa, VMMDEV_BAR1, PCI_MAPREG_TYPE_MEM,
	    BUS_SPACE_MAP_LINEAR,
	    &sc->sc_memt, &sc->sc_memh, NULL, &sc->sc_memsize)) {
		aprint_error_dev(sc->sc_dev, "can't map mem space\n");
		return;
	}

	if (vboxtime_init(sc) == -1)
		return;
	sc->sc_sync_ticks = mstohz(VBOXTIME_SYNC_INTERVAL * 1000);
	callout_setfunc(&sc->sc_sync_callout, vboxtime_sync, sc);
	vboxtime_sync(sc);
}

static int
vboxtime_detach(device_t self, int flags)
{
	struct vboxtime_softc *sc = device_private(self);

	if (sc->sc_iosize != 0)
		bus_space_unmap(sc->sc_iot, sc->sc_ioh, sc->sc_iosize);
	if (sc->sc_memsize != 0)
		bus_space_unmap(sc->sc_memt, sc->sc_memh, sc->sc_memsize);

	callout_halt(&sc->sc_sync_callout, NULL);
	callout_destroy(&sc->sc_sync_callout);

	return 0;
}

static int
vboxtime_modcmd(modcmd_t cmd, void *aux)
{
	switch (cmd) {
	case MODULE_CMD_INIT:
#ifdef _MODULE
		return config_init_component(cfdriver_ioconf_vboxtime,
		    cfattach_ioconf_vboxtime, cfdata_ioconf_vboxtime);
#else
		return 0;
#endif
	case MODULE_CMD_FINI:
#ifdef _MODULE
		return config_fini_component(cfdriver_ioconf_vboxtime,
		    cfattach_ioconf_vboxtime, cfdata_ioconf_vboxtime);
#else
		return 0;
#endif
	case MODULE_CMD_AUTOUNLOAD:
		return EBUSY;
	default:
		return ENOTTY;
	}
}

static int
vboxtime_init(struct vboxtime_softc *sc)
{
	void *membase;
	VMMDevMemory_head *vmmdev;
	VMMDevReportGuestInfo req;
	volatile VMMDevReportGuestInfo *vreq;

	/*
	 * Validate the version and size of the MMIO region.
	 */
	membase = bus_space_vaddr(sc->sc_memt, sc->sc_memh);
	vmmdev = membase;
	if (vmmdev->version != VMMDEV_MEMORY_VERSION ||
	    vmmdev->size < 32 ||
	    vmmdev->size > sc->sc_memsize) {
		aprint_error_dev(sc->sc_dev, "invalid VMM device memory "
		    "(version=%" PRIu32 " size=%" PRIu32 ")\n",
		    vmmdev->version, vmmdev->size);
		return -1;
	}

	/*
	 * Report the guest information to the host.
	 * The old interface (ReportGuestInfo) is used rather than the new
	 * one (ReportGuestInfo2) for simplicity.
	 */
	vreq = &req;
	vboxtime_make_req_header(&req.header,
	    VMMDevReq_ReportGuestInfo, sizeof(req));
	req.guest_info.interface_version = VMMDEV_VERSION;
	req.guest_info.os_type = VBOXOSTYPE_NetBSD;

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, VMMDEV_PORT_OFF_REQUEST,
	    vtophys((vaddr_t)&req));
	if (vreq->header.rc < VINF_SUCCESS) {
		aprint_error_dev(sc->sc_dev, "VMMDevReq_ReportGuestInfo: %d\n",
		    vreq->header.rc);
		return -1;
	}

	return 0;
}

static void
vboxtime_sync(void *arg)
{
	struct vboxtime_softc *sc = arg;
	VMMDevReqHostTime req;
	volatile VMMDevReqHostTime *vreq;
	struct timeval guest, host, delta;

	vreq = &req;
	vboxtime_make_req_header(&req.header,
	    VMMDevReq_GetHostTime, sizeof(req));
	req.time = UINT64_MAX;

	/*
	 * XXX vtophys() is not MI.
	 * XXX How to ensure that req in in the 32-bit space in x86_64?
	 */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, VMMDEV_PORT_OFF_REQUEST,
	    vtophys((vaddr_t)&req));
	if (vreq->header.rc < VINF_SUCCESS) {
		aprint_error_dev(sc->sc_dev, "VMMDevReq_GetHostTime: %d\n",
		    vreq->header.rc);
		goto fail;
	}
	microtime(&guest);
	host.tv_sec = req.time / 1000;
	host.tv_usec = req.time % 1000 * 1000;
	timersub(&guest, &host, &delta);
	if (delta.tv_sec >= 0 || delta.tv_usec == 0)
		aprint_normal_dev(sc->sc_dev, "delta %lld.%06u seconds\n",
		    (long long)delta.tv_sec, (unsigned int)delta.tv_usec);
	else
		aprint_normal_dev(sc->sc_dev, "delta -%lld.%06u seconds\n",
		    (long long)(-delta.tv_sec - 1),
		    (unsigned int)(1000000 - delta.tv_usec));

fail:
	callout_schedule(&sc->sc_sync_callout, sc->sc_sync_ticks);
}

static void
vboxtime_make_req_header(VMMDevRequestHeader *header,
    VMMDevRequestType type, uint32_t size)
{
	header->size = size;
	header->version = VMMDEV_REQUEST_HEADER_VERSION;
	header->request_type = type;
	header->rc = VERR_GENERAL_FAILURE;
	header->reserved1 = 0;
	header->reserved2 = 0;
}
