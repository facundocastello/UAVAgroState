#ifndef UAVAGROSTATECALIBRATION_H
#define UAVAGROSTATECALIBRATION_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/gpu/gpu.hpp>
#include "funcionesutiles.h"
#include "CommonFunctions.h"

using namespace cv;
using namespace std;

class UAVAgroStateCalibration{
public:
    void static calibrateImg(int numCornersHor, int numCornersVer, string cameraName){
                // printf("Enter number of boards: ");
        // scanf("%d", &numBoards);
        int numSquares = numCornersHor * numCornersVer;
        Size patternsize = Size(numCornersHor, numCornersVer);

        vector<string> strImgs = CommonFunctions::obtenerImagenes("Imagenes/Calibrar/");
        vector<vector<Point3f>> object_points;		//physical position of the corners in 3d space. this has to be measured by us
        vector<vector<Point2f>> image_points;		//location of corners on in the image (2d) once the program has actual physical locations and locations			
        vector<Point2f> corners; //this will be filled by the detected corners

        TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );

        vector<Point3f> obj;
        for(int j=0; j<numSquares; j++)
            obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0));

        for(int i = 0; i < strImgs.size(); i++){
            cout<<"imagen "<<i<<endl;
            Mat frame = CommonFunctions::cargarImagen(strImgs[i],1);
            vector<Mat> BGR;
            split(frame, BGR);
            frame=BGR[2];
            
            bool patternfound = findChessboardCorners(frame,patternsize,corners);
            cout<<patternfound<<endl;
            cornerSubPix(frame,corners, Size(11,11), Size(-1,-1), criteria);
            drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
            string res = "Imagenes/resultados/Calibrar/resultados" + to_string(i) + ".png";
            imwrite(res,frame);
            if(patternfound){
                image_points.push_back(corners);
                object_points.push_back(obj);
            }

        }
        //next we get ready to do the calibration, we declare variables that will hold the unknowns:

        Mat intrinsic = Mat(3, 3, CV_32FC1);
        Mat distCoeffs;
        vector<Mat> rvecs;
        vector<Mat> tvecs;

        //modify intrinsic matrix with whatever we know. camera spect ration is 1 (that's usually the case.. if not change it as required

        intrinsic.ptr<float>(0)[0] = 1;
        intrinsic.ptr<float>(1)[1] = 1;

        //(0,0) and (1,1) are focal lengths along x and y axis
        //now finally the calibration
        Mat frame = CommonFunctions::cargarImagen(strImgs[0],1);
        calibrateCamera(object_points, image_points, frame.size(),intrinsic, distCoeffs, rvecs, tvecs);
        //after this you will have the intrinsic matrix, distortion coefficients and the rotation + translation vectors.
        //intrinsic matrix and distortion coeffs are property of the camera and lens. so if you don't change it focal lengths zoom lenses you
        //can reuse it.
                                
        // string folder = "Data/Calibrar/";

        // CommonFunctions::writeMatOnFile(folder+"distCoeffs" + cameraName,distCoeffs);
        // CommonFunctions::writeMatOnFile(folder+"intrinsic" + cameraName,intrinsic);
        UAVAgroStateCalibration::storeCalibrationMat(intrinsic,distCoeffs,cameraName);

    }

    void static undistortImgs(string cameraName){
        string folder = "Data/Calibrar/";
        vector<Mat> calibrationMat =  UAVAgroStateCalibration::readCalibrationMat(cameraName);
        vector<string> strImgs = CommonFunctions::obtenerImagenes("Imagenes/Pegado/");
        for(int i = 0 ; i < strImgs.size() ; i++){
            Mat frame = CommonFunctions::cargarImagen(strImgs[i],1);
            Mat frameUndistorted;
            undistort(frame, frameUndistorted, calibrationMat[0], calibrationMat[1]);
            size_t position = strImgs[i].find_last_of("/");
            strImgs[i].erase(strImgs[i].begin(),strImgs[i].begin()+position);
            frameUndistorted = CommonFunctions::addTransparence(frameUndistorted);
            string res = "Imagenes/Undistort" + strImgs[i] + ".png";
            imwrite(res, frameUndistorted);
        }
    }


    void static storeCalibrationMat(Mat intrinsic, Mat distCoeffs, string cameraName){
        FileStorage fs("Data/Calibrar/"+cameraName+".yml", FileStorage::WRITE);
        fs << "intrinsic" << intrinsic << "distCoeffs" << distCoeffs;
        fs.release();
    }
    vector<Mat> static readCalibrationMat(string cameraName){
        FileStorage fs("Data/Calibrar/"+cameraName+".yml", FileStorage::READ);

        Mat intrinsic, distCoeffs;
        fs["intrinsic"] >> intrinsic;
        fs["distCoeffs"] >> distCoeffs;
        
        fs.release();

        return {intrinsic, distCoeffs};
    }
};

#endif