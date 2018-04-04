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
/**
 * @brief Clase utilizada para quitar la distorsión de un conjunto de imágenes.
 * 
 */
class Undistort{
public:
    /**
     * @brief Quita la distorsión de un conjunto de imágenes utilizando una matriz de transformación.
     * 
     * @param cameraName 
     */
    void static undistortImgs(string cameraName){
        struct timeval beginAll;
		gettimeofday(&beginAll, NULL);

        string folder = "Data/Calibrar/";
        vector<Mat> calibrationMat =  Calibration::readCalibrationMat(cameraName);
        vector<string> strImgs = obtenerInput();
        parallel_for_(Range(0, strImgs.size()), [&](const Range& range){
			for(int i = range.start;i < range.end ; i++){
                string strImg = CommonFunctions::obtenerUltimoDirectorio2(strImgs[i]);
                cout << CommonFunctions::stringAzul("Comenzando imagen "+strImg) + "\n";
                struct timeval begin;
                gettimeofday(&begin, NULL);
                Mat frame = CommonFunctions::cargarImagen(strImgs[i],1);
                Mat frameUndistorted;
                undistort(frame, frameUndistorted, calibrationMat[0], calibrationMat[1]);
                size_t position = strImgs[i].find_last_of("/");
                strImgs[i].erase(strImgs[i].begin(),strImgs[i].begin()+position);
                // frameUndistorted = CommonFunctions::addTransparence(frameUndistorted);
                escribirOutput(frameUndistorted,strImgs[i]);
                CommonFunctions::tiempo(begin, "Terminar " + strImg +":");
            }
        });
        CommonFunctions::tiempo(beginAll, "Terminar todo: ");
    }
    /**
     * @brief Obtiene la ubicación de las imágenes a las que se le quitará la distorsión.
     * 
     * @return vector<string> 
     */
    vector<string> static obtenerInput(){
        return CommonFunctions::obtenerImagenes("Imagenes/Undistort/input/");
    }
    /**
     * @brief Escribe las imágenes sin distorsión.
     * 
     * @param frameUndistorted 
     * @param strImg 
     */
    void static escribirOutput(Mat frameUndistorted, string strImg){
        string res = "Imagenes/Undistort/output" + strImg;
        imwrite(res, frameUndistorted);
    }
};

#endif