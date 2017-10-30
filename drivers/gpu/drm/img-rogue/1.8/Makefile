img_basedir := drivers/gpu/drm/img-rogue/1.8
include $(img_basedir)/config_kernel.mk

obj-$(CONFIG_DRM_POWERVR_ROGUE_1_8) += pvrsrvkm_1_8.o

ccflags-y += \
 -include config_kernel.h \
 -Iinclude/drm \
 -I$(img_basedir) \
 -I$(img_basedir)/include \
 -I$(img_basedir)/km \
 -D__linux__

subdir-ccflags-y += $(call cc-disable-warning,typedef-redefinition)

include $(img_basedir)/pvrsrvkm.mk
include $(img_basedir)/mt8173/Makefile
