#ifndef UTILINFORMATION_H
#define UTILINFORMATION_H

#include "Stitcher.h"
#include "CommonFunctions.h"

/**
 * @brief Calcula los indices de desempeño.
 * 
 */
class UtilInformation{
public:
     string cameraName;
    float width, height, focal, mmpx;
    void writeCameraProperties(){
        cout << "Nombre de la camara: ";
        cin >> cameraName;
        cout << "Anchura del sensor: ";
        cin >> width;
        cout << "Altura del sensor: ";
        cin >> height;
        cout << "Longitud focal: ";
        cin >> focal;
        mmpx = width/height/focal;
        FileStorage fs("Data/InformacionUtil/"+cameraName+".yml", FileStorage::WRITE);
        fs << "cameraName" << cameraName << "width" << width << "height" << height << "focal" << focal << "mmpx" << mmpx;
        fs.release();
    }
    void readCameraProperties(){
        cout << "Nombre de la camara: ";
        cin >> cameraName;
        FileStorage fs("Data/InformacionUtil/"+cameraName+".yml", FileStorage::READ);

        fs["cameraName"] >> cameraName;
        fs["width"] >> width;
        fs["height"] >> height;
        fs["focal"] >> focal;
        fs["mmpx"] >> mmpx;

        fs.release();
    }
    
    float calcularHectareas(){
        uav::Stitcher stitcher;
        string strImg = CommonFunctions::obtenerImagenes(stitcher.obtenerOutputRF().c_str())[0];
        float tamano = CommonFunctions::obtenerParametro(strImg,"tamano");
        float altura = CommonFunctions::obtenerParametro(strImg,"altura");
        float metrospx = mmpx/1000*altura;
        Mat img = CommonFunctions::cargarImagen(strImg,1);
        long int cantPix = CommonFunctions::cantPixeles(img);
        float hectareas = metrospx * metrospx * tamano * tamano * cantPix / 10000;
        return hectareas;
    }


};


#endif