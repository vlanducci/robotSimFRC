#include "Generator.h"
#include "Splines.h"

double Generator::_stepSize = 0.001;

// Calculate segmentation length (returns -1 if there was an issue)
double Generator::calculateSegLength(int node, Spline spline) {
  double totalLength = 0;
  Waypoint oldPoint, newPoint;
  oldPoint = CatmullRom::getSplinePoint((float)node, spline);

  std::cout << "[Node " << node << "-" << node+1 << "]" << std::endl;
  // std::vector<double> lengthBuffer;
  for (double t = 0.0; t < 1.0; t += _stepSize) {
    // std::cout << "d 2.2.1" << std::endl;

    newPoint = CatmullRom::getSplinePoint((float)node + t, spline);

    // std::cout << "d 2.2.2" << std::endl;
    double xrt = (newPoint.x - oldPoint.x)*(newPoint.x - oldPoint.x);
    double yrt = (newPoint.y - oldPoint.y)*(newPoint.y - oldPoint.y);
    double xyrt = (xrt+yrt);

    if (xyrt > 0) {
      totalLength = sqrt(xyrt);

      // This is just an error check to see if the value returned is infinite or nan
      // If it's either it will just output some basic debug values from the previous calculation
      if (isinf(totalLength) || isnan(totalLength)) {
        std::cout << " -- Overflow detected, Debug Below -- " << std::endl;
        std::cout << "| New waypoints x,y: (" << newPoint.x << "," << newPoint.y << ")" << std::endl;
        std::cout << "| Old waypoints x,y: (" << oldPoint.x << "," << oldPoint.y << ")" << std::endl;
        std::cout << "| t value: " << t << std::endl;
        std::cout << "| XY rt was xrt: (" << xrt << ") & yrt: (" << yrt << ")" << std::endl;
        return -1;
      }
    }

    // lengthBuffer.push_back(bufferValue);
    oldPoint = newPoint;
    printProgress(t); // display progress on calculating (because t is 0-1 value this works in our favour for being 0-100 percent complete :) )
  }

  printProgress(1); // just displays 100% at the end of the segment
  std::cout << " Complete" << std::endl;

  return totalLength; // return the final length of segment
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

  return 0;
}