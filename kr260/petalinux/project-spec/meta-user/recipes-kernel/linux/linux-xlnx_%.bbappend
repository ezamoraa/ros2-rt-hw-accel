FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append = " file://bsp.cfg"
KERNEL_FEATURES:append = " bsp.cfg"
SRC_URI += "file://user_2023-10-20-20-22-00.cfg \
            file://user_2023-11-03-19-03-00.cfg \
            file://user_2023-12-04-18-01-00.cfg \
            "

