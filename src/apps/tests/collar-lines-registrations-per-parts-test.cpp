#include <math.h>

#include <but_velodyne/CollarLinesRegistrationPerPart.h>

#include <gtest/gtest.h>

using namespace but_velodyne;
using namespace pcl;

void generate_sphere(VelodynePointCloud &sphere) {
  cv::RNG &rng(cv::theRNG());

  for(int ring = 0; ring < 16; ring++) {
    const float inclination = (ring / 16.0 * M_PI) / 4 + M_PI*3/8;

    for (float azimuth = 0.0; azimuth < 2*M_PI; azimuth += 0.01) {
      velodyne_pointcloud::VelodynePoint pt;

      const float r = 5 + rng.uniform(-0.02, 0.02);

      pt.z = r * sin(inclination) * cos(azimuth);
      pt.x = r * sin(inclination) * sin(azimuth);
      pt.y = r * cos(inclination);
      pt.ring = ring;
      pt.phase = VelodynePointCloud::horizontalAngle(pt.z, pt.x) / 360.0;

      sphere.push_back(pt);
    }
  }

  sphere.setVelodyneModel(VelodyneSpecification::VLP16);
}

vector<Eigen::Affine3f>
        distort_sphere(const VelodynePointCloud &sphere, VelodynePointCloud &distorted, const size_t parts) {
  cv::RNG &rng(cv::theRNG());
  rng.state = 113;
  vector<Eigen::Affine3f> T(parts);
  for(int i = 0; i < parts; i++) {
    T[i] = getTransformation(rng.uniform(-1.0, 1.0)*0.5, 0, rng.uniform(-1.0, 1.0)*0.5, 0, 0, 0);
  }

  vector<PhaseFilter> filters;
  bool start = true;
  for(float end_phase = 1.0 / parts / 2.0; end_phase < 0.99; end_phase += 1.0 / parts) {
    if(start) {
      filters.push_back(PhaseFilter(-2.0, end_phase));
      start = false;
    } else {
      filters.push_back(PhaseFilter(end_phase - (1.0 / parts), end_phase));
    }
  }

  for(int i = 0; i < parts; i++) {
    VelodynePointCloud part;
    filters[i].filter(sphere, part);
    transformPointCloud(part, part, T[i]);
    distorted += part;
  }

  distorted.setVelodyneModel(VelodyneSpecification::VLP16);
  return T;
}

void ASSERT_EQ_TRANSFORMATION(const Eigen::Affine3f &t1, const Eigen::Affine3f &t2) {
  const Eigen::Affine3f delta = t1.inverse() * t2;
  ASSERT_NEAR(0, Eigen::AngleAxisf(delta.rotation()).angle(), 0.1);
  ASSERT_NEAR(0, delta.translation().norm(), 0.05);
}

TEST(CollarLinesRegistrationPerPart, RegDistortedSphere) {

  const int PARTS = 4;

  VelodynePointCloud::Ptr sphere(new VelodynePointCloud);
  generate_sphere(*sphere);
  VelodyneMultiFrame::Ptr trg_frame(new VelodyneMultiFrame(vector<string>(1, "sphere"),
          vector<VelodynePointCloud::Ptr>(1, sphere), SensorsCalibration()));

  size_t previous_sphere_points = trg_frame->clouds.front()->size();
  trg_frame = trg_frame->replaceSuffixFromPreviousFrame(*trg_frame, 1.0/(PARTS*2.0));
  ASSERT_EQ(previous_sphere_points, trg_frame->clouds.front()->size());

  VelodynePointCloud::Ptr sphere_distorted(new VelodynePointCloud);
  vector<Eigen::Affine3f> distortions = distort_sphere(*trg_frame->clouds.front(), *sphere_distorted, PARTS);

  VelodyneMultiFrame src_frame(vector<string>(1, "sphere_distorted"),
          vector<VelodynePointCloud::Ptr>(1, sphere_distorted), SensorsCalibration());
  ASSERT_EQ(src_frame.clouds.front()->size(), trg_frame->clouds.front()->size());

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;

  vector<RegistrationOutcome> results;
  register_clouds_parts(src_frame, *trg_frame, registration_parameters, pipeline_parameters,
                        PARTS, true, results);

  ASSERT_EQ(distortions.size(), results.size());

  for(int i = 0; i < distortions.size(); i++) {
    ASSERT_EQ_TRANSFORMATION(distortions[i], results[i].transformation);
  }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
