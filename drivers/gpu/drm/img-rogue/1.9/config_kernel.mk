override CHROMIUMOS_KERNEL := 1
override KERNEL_DRIVER_DIR := drivers/gpu/drm/img-rogue
override METAG_VERSION_NEEDED := 2.8.1.0.3
override MIPS_VERSION_NEEDED := 2014.07-1
override PVRSRV_MODNAME := pvrsrvkm_1_9
override PVR_BUILD_DIR := mt8173_linux
override PVR_DVFS := 1
override PVR_HANDLE_BACKEND := idr
override PVR_SYSTEM := mt8173
override RGX_TIMECORR_CLOCK := mono
override SUPPORT_BUFFER_SYNC := 1
override SUPPORT_COMPUTE := 1
override SUPPORT_ES32 := 1
override SUPPORT_GPUTRACE_EVENTS := 1
override SUPPORT_SERVER_SYNC := 1
override SUPPORT_TLA := 1
ifeq ($(CONFIG_DRM_POWERVR_ROGUE_DEBUG),y)
override BUILD := debug
override PVR_BUILD_TYPE := debug
override PVR_RI_DEBUG := 1
override PVR_SERVICES_DEBUG := 1
override SUPPORT_DEVICEMEMHISTORY_BRIDGE := 1
override SUPPORT_PAGE_FAULT_DEBUG := 1
override SUPPORT_SYNCTRACKING_BRIDGE := 1
else
override BUILD := release
override PVR_BUILD_TYPE := release
endif
ifeq ($(CONFIG_DRM_POWERVR_ROGUE_PDUMP),y)
override EXTRA_PVRSRVKM_COMPONENTS := dbgdrv
override PDUMP := 1
endif
