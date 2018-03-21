#ifndef UNDISTORT_H
#define UNDISTORT_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/gpu/gpu.hpp>
#include "CommonFunctions.h"

using namespace cv;
using namespace std;

class Undistort{
public:
    void static undistortImgs(string cameraName){
        string folder = "Data/Calibrar/";
        vector<Mat> calibrationMat =  Calibration::readCalibrationMat(cameraName);
        vector<string> strImgs = CommonFunctions::obtenerImagenes("Imagenes/Undistort/input/");
        for(int i = 0 ; i < strImgs.size() ; i++){
            Mat frame = CommonFunctions::cargarImagen(strImgs[i],1);
            Mat frameUndistorted;
            undistort(frame, frameUndistorted, calibrationMat[0], calibrationMat[1]);
            size_t position = strImgs[i].find_last_of("/");
            strImgs[i].erase(strImgs[i].begin(),strImgs[i].begin()+position);
            // frameUndistorted = CommonFunctions::addTransparence(frameUndistorted);
            string res = "Imagenes/Undistort/output" + strImgs[i];
            imwrite(res, frameUndistorted);
        }
    }
};

#endif