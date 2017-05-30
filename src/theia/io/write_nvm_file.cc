// Copyright (C) 2016 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "theia/io/write_nvm_file.h"

#include <glog/logging.h>
#include <fstream>
#include <string>
#include <unordered_map>

#include "theia/sfm/reconstruction.h"
#include "theia/sfm/camera/pinhole_camera_model.h"
#include "theia/sfm/camera/pinhole_radial_tangential_camera_model.h"
#include "theia/sfm/track.h"
#include "theia/sfm/view.h"
#include "theia/util/map_util.h"

namespace theia {

// Writes an NVM file that may then be inspected with Visual SfM or other
// software packages.
bool WriteNVMFile(const std::string& nvm_filepath,
                  const Reconstruction& reconstruction) {
  std::ofstream nvm_file;
  nvm_file.open(nvm_filepath.c_str());
  if (!nvm_file.is_open()) {
    LOG(WARNING) << "Could not open nvm file for writing: " << nvm_filepath;
    return false;
  }

  // Output the NVM header.
  nvm_file << "NVM_V3 " << std::endl << std::endl;

  // Number of cameras.
  const auto& view_ids = reconstruction.ViewIds();
  nvm_file << view_ids.size() << std::endl;
  std::unordered_map<ViewId, int> view_id_to_index;
  std::unordered_map<ViewId, std::unordered_map<TrackId, int> >
      feature_index_mapping;
  // Output each camera.
  for (const ViewId view_id : view_ids) {
    const int current_index = view_id_to_index.size();
    view_id_to_index[view_id] = current_index;

    const View& view = *reconstruction.View(view_id);
    const Camera& camera = view.Camera();
    if (camera.GetCameraIntrinsicsModelType() !=
        CameraIntrinsicsModelType::PINHOLE) {
      LOG(FATAL) << "Could not add camera " << view.Name()
                 << " to the NVM output file because nvm files only "
                    "support pinhole camera models. Please remove non-pinhole "
                    "cameras from the reconstruction and try again.";
      continue;
    }

    const Eigen::Quaterniond quat(camera.GetOrientationAsRotationMatrix());
    const Eigen::Vector3d position(camera.GetPosition());
    nvm_file << view.Name() << " " << camera.FocalLength() << " " << quat.w()
             << " " << quat.x() << " " << quat.y() << " " << quat.z() << " "
             << position.x() << " " << position.y() << " " << position.z()
             << " "
             << camera.CameraIntrinsics()->GetParameter(
                    PinholeCameraModel::RADIAL_DISTORTION_1)
             << " 0" << std::endl;

    // Assign each feature in this view to a unique feature index (unique within
    // each image, not unique to the reconstruction).
    const auto& view_track_ids = reconstruction.View(view_id)->TrackIds();
    for (int i = 0; i < view_track_ids.size(); i++) {
      const TrackId track_id = view_track_ids[i];
      feature_index_mapping[view_id][track_id] = i;
    }
  }

  // Number of points.
  const auto& track_ids = reconstruction.TrackIds();
  nvm_file << track_ids.size() << std::endl;
  // Output each point.
  for (const TrackId track_id : track_ids) {
    const Track* track = reconstruction.Track(track_id);
    const Eigen::Vector3d position = track->Point().hnormalized();

    // Normalize the color.
    Eigen::Vector3i color = track->Color().cast<int>();

    nvm_file << position.x() << " " << position.y() << " " << position.z()
             << " " << color.x() << " " << color.y() << " " << color.z() << " "
             << track->NumViews() << " ";

    // Output the observations of this 3D point.
    const auto& views_observing_track = track->ViewIds();
    for (const ViewId& view_id : views_observing_track) {
      const View* view = reconstruction.View(view_id);

      // Get the feature location normalized by the principal point.
      const Camera& camera = view->Camera();
      const Feature feature =
          (*view->GetFeature(track_id)) -
          Feature(camera.PrincipalPointX(), camera.PrincipalPointY());

      const int track_index =
          FindOrDie(FindOrDie(feature_index_mapping, view_id), track_id);
      const int view_index = FindOrDie(view_id_to_index, view_id);
      nvm_file << view_index << " " << track_index << " " << feature.x() << " "
               << feature.y() << " ";
    }
    nvm_file << std::endl;
  }

  // Indicate the end of the file.
  nvm_file << "0" << std::endl;
  nvm_file.close();
  return true;
}

bool WriteCams(const std::string &cams_filepath, const Reconstruction &reconstruction)
{
  std::ofstream cams_file;
  cams_file.open(cams_filepath);
  if(!cams_file.is_open())
  {
      LOG(WARNING) << "Could not open cams file for writing: "<<cams_filepath;
  }

  const auto view_ids = reconstruction.ViewIds();
  std::unordered_map<ViewId, int> view_id_to_index;
  std::unordered_map<ViewId, std::unordered_map<TrackId, int> > feature_index_mapping;

  for(const auto& vid :view_ids)
  {
    const int current_index = view_id_to_index.size();
    view_id_to_index[vid] = current_index;
    const auto& view = *reconstruction.View(vid);
    cams_file << "\"" << vid << "\"" << std::endl;
		cams_file << "\"" << view.Name() << "\"" << std::endl;
    const auto& camera = view.Camera();
    cams_file << camera.ImageWidth() << " " << camera.ImageHeight() << std::endl;

    Matrix3x4d pmatrix;
    camera.GetProjectionMatrix(&pmatrix);
    Eigen::IOFormat fmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", "");
    cams_file << pmatrix.format(fmt) << std::endl;
    switch(camera.GetCameraIntrinsicsModelType())
    {
      case CameraIntrinsicsModelType::PINHOLE:
        cams_file << "35D " << camera.CameraIntrinsics()->GetParameter(PinholeCameraModel::RADIAL_DISTORTION_1)
				<< " " << camera.CameraIntrinsics()->GetParameter(PinholeCameraModel::RADIAL_DISTORTION_2) << std::endl;
        break;
      case CameraIntrinsicsModelType::PINHOLE_RADIAL_TANGENTIAL:
        cams_file << "357D_TD2 " << camera.CameraIntrinsics()->GetParameter(PinholeRadialTangentialCameraModel::RADIAL_DISTORTION_1)
				<< " " << camera.CameraIntrinsics()->GetParameter(PinholeRadialTangentialCameraModel::RADIAL_DISTORTION_2)
				<< " " << camera.CameraIntrinsics()->GetParameter(PinholeRadialTangentialCameraModel::RADIAL_DISTORTION_3)
				<< " " << camera.CameraIntrinsics()->GetParameter(PinholeRadialTangentialCameraModel::TANGENTIAL_DISTORTION_1)
				<< " " << camera.CameraIntrinsics()->GetParameter(PinholeRadialTangentialCameraModel::TANGENTIAL_DISTORTION_2)
				<< std::endl;
        break;
		  case  CameraIntrinsicsModelType::FISHEYE:
			  cams_file << "3I 0" << std::endl;
			  break;
      default:
        break;
    }
  }
    cams_file.close();

  return true;
}

bool WriteSparsePts(const std::string& ply_filepath, const Reconstruction& reconstruction)
{
  std::ofstream ply_file;
  ply_file.open(ply_filepath);
  if(!ply_file.is_open())
  {
      LOG(WARNING) << "Could not open cams file for writing: "<<ply_filepath;
  }

  const auto& track_ids = reconstruction.TrackIds();

  // Output each point.
  const auto nsize = track_ids.size();
  std::vector<Eigen::Vector3d> positions;
  positions.reserve(nsize);
  std::vector<Eigen::Vector3i> colors;
  colors.reserve(nsize);
  std::vector<std::vector<ViewId>> track_views;

  for (const TrackId track_id : track_ids) 
  {
    const Track* track = reconstruction.Track(track_id);
    if(!track->IsEstimated())
      continue;
    positions.emplace_back(track->Point().hnormalized());

    // Normalize the color.
    colors.emplace_back(track->Color().cast<int>());
    track_views.emplace_back(track->ViewIds().begin(),track->ViewIds().end());
  }

	ply_file << "ply" << std::endl;
	ply_file << "format ascii 1.0" << std::endl;
	ply_file << "element vertex " << positions.size() << std::endl;
	ply_file << "property float x" << std::endl;
	ply_file << "property float y" << std::endl;
	ply_file << "property float z" << std::endl;
  ply_file << "property uchar red" << std::endl;
  ply_file << "property uchar green" << std::endl;
  ply_file << "property uchar blue" << std::endl;
	ply_file << "property list uchar int visibility" << std::endl;
	ply_file << "element face 0" << std::endl;
	ply_file << "property list uchar int vertex_index" << std::endl;
	ply_file << "end_header" << std::endl;

  for(auto i=0; i < positions.size(); ++i)
  {
    const auto& pos = positions[i];
    const auto& c = colors[i];
    const auto& views = track_views[i];
    ply_file << pos.x() << " " << pos.y() << " " << pos.z()
             << " " << c.x() << " " << c.y() << " " << c.z() << " "
             << views.size() << " ";
    for (const ViewId& view_id : views) 
    {
      ply_file << view_id<<" ";
    }
  }
  ply_file.close();
	
	return true;
}

}  // namespace theia
