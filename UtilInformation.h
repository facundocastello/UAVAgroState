#ifndef UTILINFORMATION_H
#define UTILINFORMATION_H

#include "Stitcher.h"
#include "FSManager.h"
#include "CommonFunctions.h"

/**
 * @brief Calcula los indices de desempeño.
 * 
 */
class UtilInformation{
public:
     string cameraName;
    float width, height, focal, mmpx;
    /**
     * @brief Pide un nombre de camara y verifica si existe para leerla o crearla.
     * 
     */
    void selectCamera(){
        cout << "Nombre de la camara: ";
        cin >> cameraName;
        FSManager fs(cameraName+".yml","camara");
        if( fs.existeString("cameraName") ){
            cout << "La camara existe en la base de datos por lo que sus propiedades fueron cargadas en memoria. \n";
            readCameraProperties();
        }else{
            cout << "La camara no existe en la base de datos por lo que debe ingresar sus propiedades a continuación. \n";
            writeCameraProperties();
        }
    }
    /**
     * @brief Pide que se entre por command-line algunos parametros y los guarda en un yml
     * 
     */
    void writeCameraProperties(){
        cout << "Anchura del sensor en mm: ";
        cin >> width;
        cout << "Altura del sensor en mm: ";
        cin >> height;
        cout << "Longitud focal en mm: ";
        cin >> focal;
        mmpx = width/height/focal;
        FSManager fs(cameraName+".yml","camara");
        fs.appendString("cameraName",cameraName);
        fs.appendFloat("width",width);
        fs.appendFloat("height",height);
        fs.appendFloat("focal",focal);
        fs.appendFloat("mmpx",mmpx);
    }
    /**
     * @brief Lee los parametros escritos en un yml
     * 
     */
    void readCameraProperties(){
        FSManager fs(cameraName+".yml","camara");

        cameraName = fs.readString("cameraName");
        width = fs.readFloat("width");
        height = fs.readFloat("height");
        focal = fs.readFloat("focal");
        mmpx = fs.readFloat("mmpx");
    }
    /**
     * @brief Calcula las hectareas que tiene una imágen
     * 
     * @return float 
     */
    void calcularHectareas(){
        uav::Stitcher stitcher;
        string strImg = CommonFunctions::obtenerImagenes(stitcher.obtenerOutputRF().c_str())[0];
        FSManager fs(CommonFunctions::removerExtension(CommonFunctions::obtenerUltimoDirectorio2(strImg)) + ".yml", "imagen");
        float tamano = fs.readInt("tamano");
        float altura = fs.readInt("altura");
        float metrospx = mmpx/1000*altura;
        Mat img = CommonFunctions::cargarImagen(strImg,1);
        long int cantPix = CommonFunctions::cantPixeles(img);
        float hectareas = metrospx * metrospx * tamano * tamano * cantPix / 10000;
        fs.appendFloat("hectareas",hectareas);
    }
    


};


#endif