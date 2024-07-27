/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include <fstream>

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
	// create a output stream to save data
    std::ofstream MP7file, MP8file;
    MP7file.open("MP_7.csv");
  	MP7file << "detectorType,";
  	MP8file.open("MP_8.csv");
  	MP8file << "detector/descriptor, ";
  	
  	// assign the title of each column in MP_7.csv
  	for(int i = 0; i<10;i++){
      MP7file << "frame " << i+1 << "number," << "frame " << i+1 << "mean," << "frame " << i+1 << "variance,";
      MP8file << "frame " << i+1 << "number,";
    }
  	MP7file << "\n";
  	MP8file << "\n";
  
    vector<string> detectorTypeList{"HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
  	vector<string> descriptorTypeList{"BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
  	for(auto detectorType : detectorTypeList){
      	MP7file << detectorType << ",";
      	for(auto descriptorType : descriptorTypeList){
            MP8file << detectorType << "/" << descriptorType;
          	//if (descriptorType == "ORB")continue;
			if (detectorType != "AKAZE" && descriptorType == "AKAZE")continue;
    		for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++){
        	/* LOAD IMAGE INTO BUFFER */

	        	// assemble filenames for current index
        		ostringstream imgNumber;
        		imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        		string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

		        // load image from file and convert to grayscale
        		cv::Mat img, imgGray;
        		img = cv::imread(imgFullFilename);
        		cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

		        //// TASK MP.1 - ring buffer
				if(dataBuffer.size()>1)dataBuffer.erase(dataBuffer.begin()); // drop out the first image
        
      			// push image into data frame buffer
        		DataFrame frame;
        		frame.cameraImg = imgGray;
        		dataBuffer.push_back(frame);
        		cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
				
        		/// DETECT IMAGE KEYPOINTS ///
        		// extract 2D keypoints from current image
        		vector<cv::KeyPoint> keypoints; // create empty feature list for current image
      	
        		//// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        		//// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
      			detKeypoints(keypoints, imgGray, false, detectorType);

        		//// TASK MP.3 -> only keep keypoints on the preceding vehicle
        		bool bFocusOnVehicle = true;
        		cv::Rect vehicleRect(535, 180, 180, 150);
        		if (bFocusOnVehicle){
            		for(auto it = keypoints.begin(); it != keypoints.end();){
              			if(!vehicleRect.contains((*it).pt)) {
                  			keypoints.erase(it);
                		} else {
                  			++it;
                		}
            		}
        		}
          		
	          // Calculate the distribution of neighborhood size for MP.7
    	      double mean = 0, sum = 0, deviation = 0, variance = 0;
        	  for(auto keypoint : keypoints){
            	sum += keypoint.size;
          	  }
          	  mean = sum / keypoints.size();
          	  for(auto keypoint : keypoints){
            	deviation += pow((keypoint.size - mean), 2);  
          	  }
          	  variance = deviation / keypoints.size();
          	  MP7file << keypoints.size() << "," << mean << "," << variance << ",";
        	
              
              // push keypoints and descriptor for current frame to end of data buffer
        	  (dataBuffer.end() - 1)->keypoints = keypoints;
        	  cout << "#2 : DETECT KEYPOINTS done" << endl;

	          // EXTRACT KEYPOINT DESCRIPTORS //
              //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
              //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        	  cv::Mat descriptors;
        	  // string descriptorType = "BRIEF"; // BRIEF, *ORB*, FREAK, *AKAZE*, SIFT
        	  descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

        	  // push descriptors for current frame to end of data buffer
        	  (dataBuffer.end() - 1)->descriptors = descriptors;
			  
        	  cout << "#3 : EXTRACT DESCRIPTORS done" << endl;
              
              
              if (dataBuffer.size() > 1){ // wait until at least two images have been processed
	            // MATCH KEYPOINT DESCRIPTORS //
        	    vector<cv::DMatch> matches;
				string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                string descriptorType2;
                if(descriptorType=="SIFT"){
                  descriptorType2 = "DES_HOG";
                } else{
                  descriptorType2 = "DES_BINARY";
                }
				string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            	//// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            	//// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
		matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors, matches, descriptorType2, matcherType, selectorType);

            	// store matches in current data frame
            	(dataBuffer.end() - 1)->kptMatches = matches;
				
                MP8file << matches.size() << ",";
            	cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
              }
            }
        	MP7file << "\n";
      		MP8file << "\n";

          cout << "----- Detector: " << detectorType << "Descriptor: " << descriptorType << "-----"<< std::endl;
      }
      
      
    } // eof loop over all images
	MP7file.close();
    MP8file.close();
    return 0;
}
