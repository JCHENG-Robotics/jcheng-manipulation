// Query the mounted D435i's stream profiles, intrinsics and the depth->color
// extrinsics without opening any stream. Used to decide how the depth optical
// frame relates to the calibrated 1280x720 color model.
//
// Build inside the vendor container (librealsense installed under /usr/local):
//   g++ -O1 -o rs_extrinsics_probe rs_extrinsics_probe.cpp -lrealsense2
#include <librealsense2/rs.hpp>

#include <cstdio>
#include <map>
#include <string>
#include <vector>

static const char* stream_name(rs2_stream s) {
    switch (s) {
        case RS2_STREAM_DEPTH: return "depth";
        case RS2_STREAM_COLOR: return "color";
        case RS2_STREAM_INFRARED: return "infrared";
        default: return "other";
    }
}

static void print_intrinsics(const rs2::video_stream_profile& vsp) {
    const rs2_intrinsics k = vsp.get_intrinsics();
    printf("    %s idx=%d %dx%d @%dHz %s model=%d fx=%.3f fy=%.3f cx=%.3f cy=%.3f "
           "coeffs=[%g %g %g %g %g]\n",
           stream_name(vsp.stream_type()), vsp.stream_index(),
           vsp.width(), vsp.height(), vsp.fps(),
           rs2_format_to_string(vsp.format()), static_cast<int>(k.model),
           k.fx, k.fy, k.ppx, k.ppy,
           k.coeffs[0], k.coeffs[1], k.coeffs[2], k.coeffs[3], k.coeffs[4]);
}

static void print_extrinsics(const rs2::stream_profile& from,
                             const rs2::stream_profile& to) {
    try {
        const rs2_extrinsics e = from.get_extrinsics_to(to);
        printf("  extrinsics %s idx=%d -> %s idx=%d\n",
               stream_name(from.stream_type()), from.stream_index(),
               stream_name(to.stream_type()), to.stream_index());
        printf("    R = [% .9f % .9f % .9f; % .9f % .9f % .9f; % .9f % .9f % .9f]\n",
               e.rotation[0], e.rotation[1], e.rotation[2],
               e.rotation[3], e.rotation[4], e.rotation[5],
               e.rotation[6], e.rotation[7], e.rotation[8]);
        printf("    t = [%.9f %.9f %.9f] m\n",
               e.translation[0], e.translation[1], e.translation[2]);
    } catch (const rs2::error& e) {
        printf("  extrinsics %s -> %s failed: %s\n",
               stream_name(from.stream_type()), stream_name(to.stream_type()),
               e.what());
    }
}

int main() {
    rs2::context ctx;
    const rs2::device_list devices = ctx.query_devices();
    printf("devices: %u\n", devices.size());

    for (const rs2::device& dev : devices) {
        printf("device: %s sn=%s fw=%s\n",
               dev.get_info(RS2_CAMERA_INFO_NAME),
               dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
               dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));

        std::vector<rs2::stream_profile> depth_profiles;
        std::vector<rs2::stream_profile> color_profiles;
        std::map<std::string, bool> seen;

        for (const rs2::sensor& sensor : dev.query_sensors()) {
            printf("sensor: %s\n", sensor.get_info(RS2_CAMERA_INFO_NAME));
            for (const rs2::stream_profile& profile : sensor.get_stream_profiles()) {
                if (!profile.is<rs2::video_stream_profile>()) {
                    continue;
                }
                const rs2::video_stream_profile vsp =
                    profile.as<rs2::video_stream_profile>();
                char key[160];
                snprintf(key, sizeof key, "%s/%d/%dx%d@%d/%s",
                         stream_name(vsp.stream_type()), vsp.stream_index(),
                         vsp.width(), vsp.height(), vsp.fps(),
                         rs2_format_to_string(vsp.format()));
                if (seen[key]) {
                    continue;
                }
                seen[key] = true;
                try {
                    print_intrinsics(vsp);
                } catch (const rs2::error& e) {
                    printf("    %s %dx%d @%dHz %s: no intrinsics (%s)\n",
                           stream_name(vsp.stream_type()), vsp.width(),
                           vsp.height(), vsp.fps(),
                           rs2_format_to_string(vsp.format()), e.what());
                    continue;
                }
                if (vsp.stream_type() == RS2_STREAM_DEPTH &&
                    vsp.width() == 640 && vsp.height() == 480) {
                    depth_profiles.push_back(profile);
                }
                if (vsp.stream_type() == RS2_STREAM_COLOR) {
                    color_profiles.push_back(profile);
                }
            }
        }

        // Prefer the Z16 640x480x15 depth profile the deployment streams.
        rs2::stream_profile depth;
        for (const rs2::stream_profile& profile : depth_profiles) {
            const rs2::video_stream_profile vsp =
                profile.as<rs2::video_stream_profile>();
            if (vsp.format() == RS2_FORMAT_Z16 && vsp.fps() == 15) {
                depth = profile;
                break;
            }
        }
        if (!depth) {
            printf("no 640x480x15 Z16 depth profile found\n");
        } else {
            for (const rs2::stream_profile& color : color_profiles) {
                print_extrinsics(depth, color);
                print_extrinsics(color, depth);
            }
        }
    }
    return 0;
}
