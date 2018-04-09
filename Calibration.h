#ifndef CALIBRATION_H
#define CALIBRATION_H

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
/**
 * @brief Clase utilizada para la obtención de la matriz de calibración.
 * 
 */
class Calibration{
public:
    /**
     * @brief Obtiene matriz de calibración.
     * 
     * @param numCornersHor 
     * @param numCornersVer 
     * @param cameraName 
     */
    void static calibrateImg(int numCornersHor, int numCornersVer, string cameraName){
                // printf("Enter number of boards: ");
        // scanf("%d", &numBoards);
        int numSquares = numCornersHor * numCornersVer;
        Size patternsize = Size(numCornersHor, numCornersVer);

        struct timeval beginAll;
		gettimeofday(&beginAll, NULL);

        vector<string> strImgs = obtenerInput();
        vector<vector<Point3f>> object_points(strImgs.size());		//physical position of the corners in 3d space. this has to be measured by us
        vector<vector<Point2f>> image_points(strImgs.size());		//location of corners on in the image (2d) once the program has actual physical locations and locations			
        vector<vector<Point3f>> object_points2;		//physical position of the corners in 3d space. this has to be measured by us
        vector<vector<Point2f>> image_points2;		//location of corners on in the image (2d) once the program has actual physical locations and locations			
        vector<Point2f> corners; //this will be filled by the detected corners

        TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );

        vector<Point3f> obj;
        for(int j=0; j<numSquares; j++)
            obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0));

		parallel_for_(Range(0, strImgs.size()), [&](const Range& range){
			for(int i = range.start;i < range.end ; i++){

                struct timeval begin;
		        gettimeofday(&begin, NULL);
                cout << CommonFunctions::stringAzul("Buscando patrones en imagen " +strImgs[i]) + "\n";

                Mat frame = CommonFunctions::cargarImagen(strImgs[i],1);
                vector<Mat> BGR;
                split(frame, BGR);
                frame=BGR[2];
                
                bool patternfound = findChessboardCorners(frame,patternsize,corners);
                CommonFunctions::tiempo( begin , "Encontrar patrones para " + strImgs[i] + (patternfound?" con":" sin") + " éxito: ");
                cornerSubPix(frame,corners, Size(11,11), Size(-1,-1), criteria);
                drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
                if(patternfound){
                    image_points[i] = corners;
                    object_points[i] = obj;
                }
            }
        });
        for(int i = 0; i < strImgs.size(); i++){
            if( !image_points[i].empty()){
                image_points2.push_back(image_points[i]);
                object_points2.push_back(object_points[i]);                
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
        calibrateCamera(object_points2, image_points2, frame.size(),intrinsic, distCoeffs, rvecs, tvecs);
        //after this you will have the intrinsic matrix, distortion coefficients and the rotation + translation vectors.
        //intrinsic matrix and distortion coeffs are property of the camera and lens. so if you don't change it focal lengths zoom lenses you
        //can reuse it.
                                
        // string folder = "Data/Calibrar/";

        // CommonFunctions::writeMatOnFile(folder+"distCoeffs" + cameraName,distCoeffs);
        // CommonFunctions::writeMatOnFile(folder+"intrinsic" + cameraName,intrinsic);
        Calibration::storeCalibrationMat(intrinsic,distCoeffs,cameraName);

        CommonFunctions::tiempo(beginAll, "Terminar de obtener matriz de calibración: ");
    }

    /**
     * @brief Escribe matriz de calibración.
     * 
     * @param intrinsic 
     * @param distCoeffs 
     * @param cameraName 
     */
    void static storeCalibrationMat(Mat intrinsic, Mat distCoeffs, string cameraName){
        FileStorage fs("Data/Calibrar/"+cameraName, FileStorage::WRITE);
        fs << "intrinsic" << intrinsic << "distCoeffs" << distCoeffs;
        fs.release();
    }
    /**
     * @brief Lee matriz de calibración.
     * 
     * @param cameraName 
     * @return vector<Mat> 
     */
    vector<Mat> static readCalibrationMat(string cameraName){
        FileStorage fs("Data/Calibrar/"+cameraName, FileStorage::READ);

        Mat intrinsic, distCoeffs;
        fs["intrinsic"] >> intrinsic;
        fs["distCoeffs"] >> distCoeffs;
        
        fs.release();

        return {intrinsic, distCoeffs};
    }
    /**
     * @brief Obtiene las ubicaciones de las imágenes de entrada.
     * 
     * @return vector<string> 
     */
    vector<string> static obtenerInput(){
        return CommonFunctions::obtenerImagenes("Imagenes/Calibrar/input/");
    }
};

#endif