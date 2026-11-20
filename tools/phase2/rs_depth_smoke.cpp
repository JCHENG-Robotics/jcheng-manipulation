// Depth-stream smoke test: open the D435i depth sensor through librealsense,
// stream N frames, and report validity/extent/intrinsics. Color is left alone so
// this can run while /dev/video4 is still owned by the UVC colour node.
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

int main() {
    try {
        rs2::pipeline pipe;
        rs2::config cfg;
        // Depth alone yields no URBs on this kernel/firmware; the D400 ASIC only
        // starts delivering once the IR stream is running too.
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
        if (getenv("RS_SMOKE_IR"))
            cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 15);
        auto profile = pipe.start(cfg);

        auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
        printf("depth scale: %.6f m/unit\n", depth_sensor.get_depth_scale());
        auto intr = profile.get_stream(RS2_STREAM_DEPTH)
                        .as<rs2::video_stream_profile>()
                        .get_intrinsics();
        printf("intrinsics: fx=%.3f fy=%.3f ppx=%.3f ppy=%.3f w=%d h=%d model=%d\n",
               intr.fx, intr.fy, intr.ppx, intr.ppy, intr.width, intr.height, (int)intr.model);

        int frames = 0;
        for (int i = 0; i < 10; i++) {
            rs2::frameset fs;
            try {
                fs = pipe.wait_for_frames(3000);
            } catch (const rs2::error& e) {
                printf("wait_for_frames failed on frame %d: %s\n", i + 1, e.what());
                break;
            }
            rs2::depth_frame d = fs.get_depth_frame();
            if (!d) continue;
            frames++;
            const uint16_t* px = (const uint16_t*)d.get_data();
            size_t n = (size_t)d.get_width() * d.get_height();
            size_t valid = 0;
            uint16_t lo = 0xffff, hi = 0;
            for (size_t k = 0; k < n; k++) {
                if (!px[k]) continue;
                valid++;
                lo = std::min(lo, px[k]);
                hi = std::max(hi, px[k]);
            }
            float scale = depth_sensor.get_depth_scale();
            printf("frame %d: %zu/%zu valid (%.1f%%), z %.3f..%.3f m, ts=%.3f\n", frames, valid, n,
                   100.0 * valid / n, lo * scale, hi * scale, d.get_timestamp());
        }
        pipe.stop();
        printf("frames=%d\n", frames);
        return frames > 0 ? 0 : 2;
    } catch (const rs2::error& e) {
        printf("rs2 error: %s\n", e.what());
        return 1;
    }
}
