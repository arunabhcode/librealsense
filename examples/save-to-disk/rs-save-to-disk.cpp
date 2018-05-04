// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API

#include <fstream>   // File IO
#include <iostream>  // Terminal IO
#include <sstream>   // Stringstreams

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// Helper function for writing metadata to disk as a csv file
void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

// This sample captures 30 frames and writes the last frame to disk.
// It can be useful for debugging an embedded system with no display.
int main(int argc, char* argv[]) try
{
  // Declare depth colorizer for pretty visualization of depth data
  rs2::colorizer color_map;
  color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2);

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::config config;
  config.enable_stream(
      rs2_stream::RS2_STREAM_COLOR, 1280, 720, rs2_format::RS2_FORMAT_RGB8, 30);
  config.enable_stream(
      rs2_stream::RS2_STREAM_DEPTH, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);
  rs2::pipeline pipe;
  // Start streaming with default recommended configuration
  pipe.start(config);

  // Capture 30 frames to give autoexposure, etc. a chance to settle
  for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();

  // Wait for the next set of frames from the camera. Now that autoexposure,
  // etc. has settled, we will write these to disk
  rs2::frameset frames = pipe.wait_for_frames();
  for (auto&& frame : frames)
  {
    // We can only save video frames as pngs, so we skip the rest
    if (auto vf = frame.as<rs2::video_frame>())
    {
      auto stream = frame.get_profile().stream_type();
      // Use the colorizer to get an rgb image for the depth stream
      if (vf.is<rs2::depth_frame>()) vf = color_map(frame);

      // Write images to disk
      std::stringstream png_file;
      png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
               << ".png";
      stbi_write_png(png_file.str().c_str(),
                     vf.get_width(),
                     vf.get_height(),
                     vf.get_bytes_per_pixel(),
                     vf.get_data(),
                     vf.get_stride_in_bytes());
      std::cout << "Saved " << png_file.str() << std::endl;

      // Record per-frame metadata for UVC streams
      std::stringstream csv_file;
      csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name()
               << "-metadata.csv";
      metadata_to_csv(vf, csv_file.str());
    }
    // if (vf.is<rs2::depth_frame>())
    // {
    // }
  }
  rs2::depth_frame depth = frames.get_depth_frame();
  rs2::video_frame color = frames.get_color_frame();
  std::cout << color.get_width() << " " << color.get_height() << std::endl;
  std::cout << depth.get_width() << " " << depth.get_height() << std::endl;
  rs2::pointcloud pc;
  pc.map_to(color);
  rs2::points points = pc.calculate(depth);
  std::string out_pc = "rs-save-to-disk-output-Colored-Pointcloud.ply";
  std::cout << "Saved " << out_pc << std::endl;
  points.export_to_ply(out_pc.c_str(), color);
  return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
  std::ofstream csv;

  csv.open(filename);

  //    std::cout << "Writing metadata to " << filename << endl;
  csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type())
      << "\nMetadata Attribute,Value\n";

  // Record all the available metadata attributes
  for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i++)
  {
    if (frm.supports_frame_metadata((rs2_frame_metadata_value)i))
    {
      csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
          << frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
    }
  }

  csv.close();
}
