// Internal probe: call librealsense's own discovery helpers to find where the
// D435i is dropped (uvc list -> filter_by_product -> grouping -> mi_present -> pick).
// Compiled against the internal headers of the 2.55.1 source tree; every symbol
// used here is exported from librealSense2.so (Release, default visibility).
//
// Build (inside the vendor container, source tree at /home/ubuntu/librealsense-2.55.1):
//   g++ -O1 -std=c++14 -DRS2_USE_V4L2_BACKEND -DUSING_UDEV -DBUILD_SHARED_LIBS \
//     -o rs_internal_probe rs_internal_probe.cpp \
//     -I/home/ubuntu/librealsense-2.55.1 -I/home/ubuntu/librealsense-2.55.1/src \
//     -I/home/ubuntu/librealsense-2.55.1/include \
//     -I/home/ubuntu/librealsense-2.55.1/third-party/rsutils/include \
//     -I/home/ubuntu/lrs_build/third-party/json/include \
//     -I/usr/local/include -L/usr/local/lib -lrealsense2 -Wl,-rpath,/usr/local/lib
#include <cstring>
#include <src/linux/backend-v4l2.h>
#include <src/platform/platform-utils.h>
#include <src/ds/d400/d400-info.h>
#include <src/ds/d400/d400-private.h>
#include <src/context.h>
#include <librealsense2/rs.hpp>

#include <iostream>

using namespace librealsense;

int main() {
    platform::v4l_backend be;

    auto uvc = be.query_uvc_devices();
    std::cout << "query_uvc_devices: " << uvc.size() << "\n";
    for (auto&& i : uvc) {
        std::cout << "---\n" << i.operator std::string();
        std::cout << "  mi_present(0)=" << platform::mi_present({i}, 0)
                  << " meta=" << i.has_metadata_node << "\n";
    }
    auto by_pid = platform::filter_by_product(uvc, ds::rs400_sku_pid);
    std::cout << "filter_by_product(rs400_sku_pid): " << by_pid.size() << "\n";
    auto groups = platform::group_devices_by_unique_id(by_pid);
    std::cout << "groups: " << groups.size() << "\n";
    for (auto&& g : groups) {
        std::cout << "  group unique_id=" << (g.empty() ? "<empty>" : g.front().unique_id)
                  << " n=" << g.size() << " mi_present(0)=" << platform::mi_present(g, 0)
                  << " mi_present(3)=" << platform::mi_present(g, 3) << "\n";
    }

    auto usb = be.query_usb_devices();
    auto hid = be.query_hid_devices();
    std::cout << "usb=" << usb.size() << " hid=" << hid.size() << "\n";
    for (auto&& u : usb)
        std::cout << "  usb id=" << u.id << " vid=" << std::hex << u.vid << " pid=" << u.pid
                  << std::dec << " unique_id=" << u.unique_id << "\n";

    std::shared_ptr<context> ctx = context::make((char const*)nullptr);
    platform::backend_device_group group(uvc, usb, hid);
    auto picked = d400_info::pick_d400_devices(ctx, group);
    std::cout << "pick_d400_devices: " << picked.size() << "\n";
    std::cout << "group.uvc_devices after pick: " << group.uvc_devices.size() << "\n";

    // Replica of d400_info::pick_d400_devices using the library's own helpers,
    // to see whether the binary behaves like the source we are reading.
    {
        auto valid_pid = platform::filter_by_product(uvc, ds::rs400_sku_pid);
        auto gd = platform::group_devices_and_hids_by_unique_id(
            platform::group_devices_by_unique_id(valid_pid), hid);
        int replica = 0;
        for (auto&& g : gd) {
            auto& devices = g.first;
            bool all_sensors_present = platform::mi_present(devices, 0);
            bool multi = false;
            for (auto&& u : devices)
                if (std::find(std::begin(ds::d400_multi_sensors_pid),
                              std::end(ds::d400_multi_sensors_pid), u.pid) != std::end(ds::d400_multi_sensors_pid))
                    multi = true;
            if (multi) {
                all_sensors_present = all_sensors_present && platform::mi_present(devices, 3);
                all_sensors_present = true;
            }
            std::cout << "replica: group n=" << devices.size() << " multi=" << multi
                      << " mi0=" << platform::mi_present(devices, 0) << " => pick="
                      << ((!devices.empty() && all_sensors_present) ? 1 : 0) << "\n";
            if (!devices.empty() && all_sensors_present) replica++;
        }
        std::cout << "replica picked: " << replica << "\n";
    }
    return 0;
}
