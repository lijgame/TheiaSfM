
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <theia/theia.h>

#include <string>

DEFINE_string(output_cams, "", "Output cams file.");
DEFINE_string(output_ply, "", "Output ply file.");
DEFINE_string(reconstruction, "", "Input reconstruction file in binary format.");

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Load the reconstuction.
  theia::Reconstruction reconstruction;
  CHECK(theia::ReadReconstruction(FLAGS_reconstruction,
                                  &reconstruction))
      << "Could not read Reconstruction file.";

  CHECK(theia::WriteCams(FLAGS_output_cams, reconstruction))
      << "Could not write cams file.";

  CHECK(theia::WriteSparsePts(FLAGS_output_ply, reconstruction))
      << "Could not write cams file.";
  return 0;
}
