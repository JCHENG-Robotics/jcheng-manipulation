// Diagnostic probe: why does librealsense see zero RealSense devices?
// Prints host udev state, the exact enumeration librealsense performs
// (subsystem "usb" + DEVTYPE match), and an rs2::context query with debug logs.
// Build: g++ -O1 -o rs_probe rs_probe.cpp -lrealsense2 -ludev
#include <libudev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cerrno>
#include <librealsense2/rs.hpp>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace std;

static void print_udev_node(udev* u, const char* syspath) {
    udev_device* d = udev_device_new_from_syspath(u, syspath);
    if (!d) {
        printf("  %s  <udev_device_new_from_syspath failed>\n", syspath);
        return;
    }
    const char* vid = udev_device_get_sysattr_value(d, "idVendor");
    const char* pid = udev_device_get_sysattr_value(d, "idProduct");
    const char* bus = udev_device_get_sysattr_value(d, "busnum");
    const char* addr = udev_device_get_sysattr_value(d, "devnum");
    const char* devtype = udev_device_get_property_value(d, "DEVTYPE");
    const char* product = udev_device_get_property_value(d, "ID_MODEL_FROM_DATABASE");
    if (!product) product = udev_device_get_sysattr_value(d, "product");
    printf("  %s  vid=%s pid=%s bus=%s addr=%s DEVTYPE=%s product=%s\n", syspath,
           vid ? vid : "-", pid ? pid : "-", bus ? bus : "-", addr ? addr : "-",
           devtype ? devtype : "-", product ? product : "-");
    udev_device_unref(d);
}

// Same query librealsense's linux usb-enumerator performs.
static int enumerate(udev* u, bool match_devtype, const char* label) {
    udev_enumerate* e = udev_enumerate_new(u);
    if (!e) { printf("%s: udev_enumerate_new failed\n", label); return -1; }
    udev_enumerate_add_match_subsystem(e, "usb");
    if (match_devtype) udev_enumerate_add_match_property(e, "DEVTYPE", "usb_device");
    int rc = udev_enumerate_scan_devices(e);
    if (rc < 0) printf("%s: scan_devices rc=%d (%s)\n", label, rc, strerror(-rc));
    int n = 0;
    udev_list_entry* devs = udev_enumerate_get_list_entry(e);
    udev_list_entry* entry;
    udev_list_entry_foreach(entry, devs) {
        const char* syspath = udev_list_entry_get_name(entry);
        if (n < 40) print_udev_node(u, syspath);
        n++;
    }
    printf("%s: %d device(s)\n", label, n);
    udev_enumerate_unref(e);
    return n;
}

// V4L2 node capabilities, as librealsense reads them (device_caps, not caps)
static void print_v4l2_caps() {
    printf("=== v4l2 node caps (VIDIOC_QUERYCAP) ===\n");
    for (int i = 0; i < 8; i++) {
        char path[32];
        snprintf(path, sizeof path, "/dev/video%d", i);
        int fd = open(path, O_RDWR | O_NONBLOCK);
        bool ro = false;
        if (fd < 0) { fd = open(path, O_RDONLY | O_NONBLOCK); ro = true; }
        if (fd < 0) continue;
        struct v4l2_capability cap;
        memset(&cap, 0, sizeof cap);
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
            printf("%s card=%s bus=%s caps=0x%08x device_caps=0x%08x meta=%s opened=%s\n",
                   path, cap.card, cap.bus_info, cap.capabilities, cap.device_caps,
                   (cap.device_caps & V4L2_CAP_META_CAPTURE) ? "YES" : "no",
                   ro ? "ro" : "rw");
        } else {
            printf("%s QUERYCAP failed errno=%d (%s) opened=%s\n", path, errno, strerror(errno),
                   ro ? "ro" : "rw");
        }
        close(fd);
    }
}

int main() {
    printf("=== udev runtime ===\n");
    FILE* f = fopen("/run/udev/data/../control", "r");
    printf("/run/udev reachable: %s\n", f ? "yes" : "no");
    if (f) fclose(f);
    udev* u = udev_new();
    if (!u) { printf("udev_new() FAILED\n"); return 1; }
    printf("udev_new() ok\n");

    printf("\n=== usb + DEVTYPE=usb_device (librealsense query) ===\n");
    int matched = enumerate(u, true, "matched");

    printf("\n=== usb, no property filter ===\n");
    int all = enumerate(u, false, "all");
    udev_unref(u);

    print_v4l2_caps();

    printf("\n=== rs2::context ===\n");
    rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        printf("rs2 devices: %zu\n", devices.size());
        for (auto&& dev : devices) {
            printf("  %s sn=%s\n", dev.get_info(RS2_CAMERA_INFO_NAME),
                   dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        }
    } catch (const rs2::error& e) {
        printf("rs2 exception: %s\n", e.what());
    }

    printf("\nSUMMARY matched=%d all=%d\n", matched, all);
    return 0;
}
