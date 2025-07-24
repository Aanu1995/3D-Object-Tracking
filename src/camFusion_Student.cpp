
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT) {
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
        // assemble vector for matrix-vector-multiplication by converting to homogenous coordinate
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;

        cv::Point pt {};
        // pixel coordinates. Convert from homogenous coordinate to euclidean coordinate in 2D
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes {}; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2) {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges

            cv::Rect smallerBox {};
            smallerBox.x = it2->roi.x + (shrinkFactor * (it2->roi.width / 2.0));
            smallerBox.y = it2->roi.y + (shrinkFactor * (it2->roi.height / 2.0));
            smallerBox.width = it2->roi.width * (1 - shrinkFactor);
            smallerBox.height = it2->roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt)) {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1) {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/*
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size.
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0;
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2) {
            // world coordinates
            float xw = it2->x; // world position in m with x facing forward from sensor
            float yw = it2->y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates. Converting from world to image coordinate
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        std::ostringstream oss1, oss2;
        oss1 << "id=" << it1->boxID << ", #pts=" << static_cast<int>(it1->lidarPoints.size());
        putText(topviewImg, oss1.str(), cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        oss2 << std::fixed << std::setprecision(2);
        oss2 << "xmin=" << xwmin << " m, yw=" << (ywmax-ywmin) << " m";
        putText(topviewImg, oss2.str(), cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i) {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait) {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches){

    // Get all matches where the current keypoint is within the bounding box ROI
    std::vector<cv::DMatch> matchesInROI {};
    std::vector<double> distances {};

    for (const auto& match : kptMatches) {
        cv::Point2f currPt = kptsCurr[match.trainIdx].pt;
        if (boundingBox.roi.contains(currPt)) {
            matchesInROI.push_back(match);
            // Compute distance between matched keypoints
            cv::Point2f prevPt = kptsPrev[match.queryIdx].pt;
            distances.push_back(cv::norm(currPt - prevPt));
        }
    }

    // Filter out outlier matches using distance statistics (e.g., remove matches with distance > mean + 1.5*stddev)
    if (!distances.empty()) {
        double mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        // Calculate the sum of squared distances between keypoint matches.
        // This value is needed to compute the variance and standard deviation of the distances,
        // which helps us identify and filter out outlier matches.
        double sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / (distances.size() - std::pow(mean, 2)));

        for (size_t i = 0; i < matchesInROI.size(); ++i) {
            if (std::abs(distances[i] - mean) < 1.5 * stdev) {
                boundingBox.kptMatches.push_back(matchesInROI[i]);
            }
        }
    }
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC) {

    const double minDist = 100.0; // Minimum required distance
    std::vector<double> distRatios {};

    for (size_t i = 0; i < kptMatches.size(); ++i) {
        cv::KeyPoint kpOuterCurr = kptsCurr[kptMatches[i].trainIdx];
        cv::KeyPoint kpOuterPrev = kptsPrev[kptMatches[i].queryIdx];

        for (size_t j = i + 1; j < kptMatches.size(); ++j) {
            cv::KeyPoint kpInnerCurr = kptsCurr[kptMatches[j].trainIdx];
            cv::KeyPoint kpInnerPrev = kptsPrev[kptMatches[j].queryIdx];

            // Compute distances between keypoints in previous and current frame
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            // Avoid division by zero
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist) {
                double distRatio = distCurr / distPrev;
                distRatios.emplace_back(distRatio);
            }
        }
    }

    if (distRatios.size() == 0) {
        TTC = NAN;
        return;
    }

    // Use median to mitigate outlier influence
    std::sort(distRatios.begin(), distRatios.end());

    int medianIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0? (distRatios[medianIndex - 1] + distRatios[medianIndex]) / 2.0 : distRatios[medianIndex];

    // Compute TTC
    double dT = 1.0 / frameRate; // (time period = 1/frequency)
    if (medDistRatio != 1.0) {
        TTC = -dT / (1.0 - medDistRatio);
    } else {
        TTC = NAN;
    }
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC) {

    // To compute the TTC, the forward direction (distance in driving direction) of the car is considered
    // Extract x values from Lidar points (distance in driving direction)

    std::vector<double> prevXs{};
    std::vector<double> currXs {};

    for (const auto& pt : lidarPointsPrev) {
        prevXs.push_back(pt.x);
    }

    for (const auto& pt : lidarPointsCurr) {
        currXs.push_back(pt.x);
    }

    // sort in ascending order to calculate the median
    std::sort(prevXs.begin(), prevXs.end());
    std::sort(currXs.begin(), currXs.end());

    // Use the median x-value of lidar points (not the minimum), to avoid outliers
    // that are unrealistically close and would distort the TTC estimate
    int prevMedianIndex = floor(prevXs.size() / 2.0);
    double medianPrevX = prevXs.size() % 2 == 0? (prevXs[prevMedianIndex - 1] + prevXs[prevMedianIndex]) / 2.0 : prevXs[prevMedianIndex];
    int currMedianIndex = floor(currXs.size() / 2.0);
    double medianCurrX = currXs.size() % 2 == 0? (currXs[currMedianIndex - 1] + currXs[currMedianIndex]) / 2.0 : currXs[currMedianIndex];

    // Compute TTC
    double dT = 1.0 / frameRate; // (time period = 1/frequency)
    if (medianPrevX - medianCurrX > 0) {
        TTC = (medianCurrX * dT) / (medianPrevX - medianCurrX);
    } else {
        TTC = NAN; // Avoid division by zero
    }
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame) {
    // Map to count matches between bounding boxes: [prevBoxID][currBoxID] = count
    map<int, map<int, int>> boxMatchCounts {};

    // For each keypoint match between the prev frame and current frame, determine which bounding boxes the keypoints belong to
    for(const auto& match : matches) {

        std::vector<int> prevBoxIDs{};
        std::vector<int> currBoxIDs{};

        // Find all bounding boxes in previous frame containing the keypoint
        for (const auto& box : prevFrame.boundingBoxes) {
            if (box.roi.contains(prevFrame.keypoints[match.queryIdx].pt)) {
                prevBoxIDs.push_back(box.boxID);
            }
        }

        // Find all bounding boxes in current frame containing the keypoint
        for (const auto& box : currFrame.boundingBoxes) {
            if (box.roi.contains(currFrame.keypoints[match.trainIdx].pt)) {
                currBoxIDs.push_back(box.boxID);
            }
        }

        // Count how many keypoint matches connect each previous box to each current box
        for (int prevID : prevBoxIDs) {
            for (int currID : currBoxIDs) {
                boxMatchCounts[prevID][currID] += 1;
            }
        }
    }

    // For each bounding box in previous frame, find the best matching box in current frame
    for (const auto& prevPair : boxMatchCounts) {
        int bestCurrBoxID = -1;
        int maxCount = 0;
        for (const auto& currPair : prevPair.second) {
            if (currPair.second > maxCount) {
                maxCount = currPair.second;
                bestCurrBoxID = currPair.first;
            }
        }

        if (bestCurrBoxID >= 0) {
            bbBestMatches[prevPair.first] = bestCurrBoxID;
        }
    }

}
