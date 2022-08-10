#pragma once

#include <opencv2/opencv.hpp>
#include <string>

// Reads a .dpt file (Sintel format) into an OpenCV matrix.
cv::Mat1f readSintelDptFile(const char* filename);

// Reads a .dpt file (Sintel format) into an OpenCV matrix.
cv::Mat1f readSintelDptFile(std::string filename);


// Writes a depth map to a .dpt file (Sintel format).
void writeSintelDptFile(const char* filename, cv::Mat1f img);

// Writes a depth map to a .dpt file (Sintel format).
void writeSintelDptFile(std::string filename, cv::Mat1f img);