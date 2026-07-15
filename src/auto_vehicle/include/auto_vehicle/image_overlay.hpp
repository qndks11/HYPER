#ifndef IMAGE_OVERLAY_HPP
#define IMAGE_OVERLAY_HPP

#include <opencv2/opencv.hpp>

// Copies every pixel of `overlay` that isn't pure black onto `base`, in place. Lets a detector's
// annotations -- mask highlight, chain points, fitted lines, text, ... -- be drawn once onto an
// otherwise-black canvas the same size as the BEV frame, then reused both for that node's own
// debug view (composed onto its own BEV frame) and, unmodified, published for visualizer to
// stack multiple detectors' annotations onto one shared frame without each one's black
// background blotting out what's already there.
inline void compose_overlay(cv::Mat & base, const cv::Mat & overlay)
{
  cv::Mat channels[3];
  cv::split(overlay, channels);
  const cv::Mat nonzero = (channels[0] > 0) | (channels[1] > 0) | (channels[2] > 0);
  overlay.copyTo(base, nonzero);
}

#endif  // IMAGE_OVERLAY_HPP
