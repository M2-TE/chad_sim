#include <fstream>
#include <rmagine/types/sensors.h>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

int main() {
    // load map
    rmagine::EmbreeMapPtr map = rmagine::import_embree_map("sponza/sponza.obj");

    // simulate lidar
    rmagine::SphericalModel model {
        .phi {
            .min = -15.0 * M_PI / 180.0,
            .inc = 2.0 * M_PI / 180.0,
            .size = 16,
        },
        .theta {
            .min = -M_PI,
            .inc = 0.4 * M_PI / 180.0,
            .size = 900,
        },
        .range {
            .min = 0.0,
            .max = 100.0,
        },
    };
    // or just use preset
    model = rmagine::vlp16_900();


    // prepare poses
    rmagine::Memory<rmagine::Transform, rmagine::RAM> poses(1000);
    for(size_t i = 0; i < poses.size(); i++) {
        rmagine::Transform T;
        T.t = {-10.0, 3.0, 0.0}; // start at the back, slightly above ground
        T.t.x += float(i)*0.02f; // move forward
        T.R.set({0.0f, 0.1f*float(i), 0.0f}); // spin around like a maniac
        poses[i] = T;
    }

    // simulate scan session
    using ResultT = rmagine::Bundle<rmagine::Hits<rmagine::RAM>, rmagine::Points<rmagine::RAM>>;
    rmagine::SphereSimulatorEmbree sim;
    sim.setMap(map);
    sim.setModel(model);
    ResultT result = sim.simulate<ResultT>(poses);

    // write points for testing
    std::ofstream ofs("points.asc");
    for (int ray_i = 0; ray_i < result.hits.size(); ray_i++) {
        if (!result.hits[ray_i]) continue;
        
        // figure out corresponding pose index
        size_t rays_per_scan = result.hits.size() / poses.size();
        size_t pose_i = ray_i / rays_per_scan;

        // apply pose transform to the point
        auto& T = poses[pose_i];
        auto point = T.mult(result.points[ray_i]);

        ofs << point.x << ' '
            << point.y << ' '
            << point.z << '\n';
    }
}