#include "Generator.h"
#include "Splines.h"

double Generator::_stepSize = 0.001;
double Generator::splineLength = 0;

// Calculate segmentation length (returns -1 if there was an issue)
double Generator::calculateSegLength(int node, Spline spline) {
  double segLength = 0;
  SplinePoint oldPoint, newPoint;
  oldPoint = CatmullRom::getSplinePoint((float)node, spline);

  std::cout << "[Node " << node << "-" << node+1 << "]" << std::endl;
  for (double t = 0.0; t < 1.0; t += _stepSize) {
    newPoint = CatmullRom::getSplinePoint((float)node + t, spline);

    // 1 means end of spline so return
    if (newPoint.flag != 0 || oldPoint.flag != 0) {
      printProgress(1);
      std::cout << " Complete" << std::endl;
      return segLength;
    }

    double xrt = (newPoint.waypoint.x - oldPoint.waypoint.x)*(newPoint.waypoint.x - oldPoint.waypoint.x);
    double yrt = (newPoint.waypoint.y - oldPoint.waypoint.y)*(newPoint.waypoint.y - oldPoint.waypoint.y);
    double xyrt = (xrt+yrt);
    
    double bufferLength = 0;

    if (xyrt > 0) {
      bufferLength = sqrt(xyrt);
      if (isinf(bufferLength) || isnan(bufferLength)) {
        bufferLength = 0;
        std::cout << " -- Overflow detected, Debug Below -- " << std::endl;
        std::cout << "| New points x,y: (" << (double)newPoint.waypoint.x << "," << (double)newPoint.waypoint.y << ")" << std::endl;
        std::cout << "| Old points x,y: (" << oldPoint.waypoint.x << "," << oldPoint.waypoint.y << ")" << std::endl;
        std::cout << "| t value: " << t << std::endl;
        std::cout << "| XY rt was xrt: (" << xrt << ") & yrt: (" << yrt << ")" << std::endl;
        return -1;
      }
    } else {
      bufferLength = 0;
    }

    segLength += bufferLength;
    oldPoint = newPoint;
    printProgress(t);
  }

  return segLength;
}

/**
 * Build spline,
 * The input of the spline is a reference (&) so it changes the input spline rather than
 * creating a new one and outputing it.
 * Instead the function returns an int, -1 or 1 depending on success or failure
 * 
 * Use remove nodes to remove nodes to remove a number of nodes from the front and back of spline
 * (Should be useless because the algorithm does this for you. But there anyway to play around with)
 */
int Generator::buildPath(Spline &spline, int removeNodes) {
  int nodeNum = spline.waypoints.size();
  std::cout << "-- Calculating Length of spline --" << std::endl;
  std::cout << "-- Total Nodes: " << nodeNum << std::endl;
  for (size_t node = removeNodes; node < nodeNum - removeNodes; node++) {
    double segLength = calculateSegLength(node, spline);
    if (segLength == -1) {
      std::cout << "Segment Length Error" << std::endl;
      return -1;
    } else {

      spline.waypoints[node].segLength = segLength;
      spline.totalLength += segLength;
      spline.waypoints[node].totalLength = spline.totalLength;
      if (spline.waypoints[node].segLength > 0) {
        spline.segmentNum++;
        std::cout << "Segment " << node << "-" << node+1 << ", Length: " << spline.waypoints[node].segLength << ", Length up to and including: " << spline.waypoints[node].totalLength << std::endl; 
      }
    }
  }

  std::cout << "Number of segments: " << spline.segmentNum << std::endl;
  splineLength = spline.totalLength;
  // std::cout << "\n\nTotal Length: " << splineLength << std::endl;
  return 0;
}