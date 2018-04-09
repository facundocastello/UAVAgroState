#ifndef PERFORMANCE_H
#define PERFORMANCE_H

#include "CommonFunctions.h"

/**
 * @brief Maneja la metadata de la camara e imágenes
 * 
 */
class FSManager{
public:
    string dir;
    /**
     * @brief Construye un fsmanager y para esto crea el archivo de 'dir' si es que no existe
     * 
     * @param dir 
     * @param tipo 
     */
     FSManager(string dir,string tipo, bool sobreescribir=false){
         dir += ".yml";
         this-> dir = dir;
         if("camara"==tipo){
            this-> dir  = "Data/Camaras/" + dir;}
         if("imagen"==tipo){
            this-> dir  = "Data/Imagenes/" + dir;}

         if(!CommonFunctions::existFile(this-> dir ) || sobreescribir){
            CommonFunctions::manejarCarpeta(this-> dir );
            FileStorage fsW(this-> dir ,FileStorage::WRITE);
            fsW << "ubicacion" << this-> dir ;
            fsW.release();
         }
     }
    
    /**
     * @brief Agrega un entero al fs con el nombre y valor ingresado
     * 
     * @param nombre 
     * @param value 
     */
    void appendInt(string nombre, int value){
       if(!existeInt(nombre)){
           FileStorage fsA(dir, FileStorage::APPEND);
           fsA << nombre << value;
           fsA.release();
        }
    }
    /**
     * @brief Agrega un flotante al fs con el nombre y valor ingresado
     * 
     * @param nombre 
     * @param value 
     */
    void appendFloat(string nombre, float value){
        if(!existeFloat(nombre)){
            FileStorage fsA(dir, FileStorage::APPEND);
            fsA << nombre << value;
            fsA.release();
        }
    }
    /**
     * @brief Agrega un string al fs con el nombre y valor ingresado
     * 
     * @param nombre 
     * @param value 
     */
    void appendString(string nombre, string value){
        if(!existeString(nombre)){
            FileStorage fsA(dir, FileStorage::APPEND);
            fsA << nombre << value;
            fsA.release();
        }
    }
    /**
     * @brief Agrega un vector<int> al fs con el nombre y valor ingresado
     * 
     * @param nombre 
     * @param value 
     */
    void appendVInt(string nombre, vector<int> value){
       if(!existeVInt(nombre)){
           FileStorage fsA(dir, FileStorage::APPEND);
           fsA << nombre << value;
           fsA.release();
        }
    }
    /**
     * @brief Agrega un vector<float> al fs con el nombre y valor ingresado
     * 
     * @param nombre 
     * @param value 
     */
    void appendVFloat(string nombre, vector<float> value){
       if(!existeVFloat(nombre)){
           FileStorage fsA(dir, FileStorage::APPEND);
           fsA << nombre << value;
           fsA.release();
        }
    }
    /**
     * @brief Lee un entero del fs con el nombre ingresado
     * 
     * @param nombre 
     * @return int 
     */
    int readInt(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        int ret = (int)fsR[nombre];
        fsR.release();
        return ret;
    }
    /**
     * @brief Lee un flotante del fs con el nombre ingresado
     * 
     * @param nombre 
     * @return int 
     */
    float readFloat(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        float ret = (float)fsR[nombre];
        fsR.release();
        return ret;
    }
    /**
     * @brief Lee un string del fs con el nombre ingresado
     * 
     * @param nombre 
     * @return int 
     */
    string readString(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        string ret = (string)fsR[nombre];
        fsR.release();
        return ret;
    }
    /**
     * @brief Lee un  vector de strings del fs con el nombre ingresado
     * 
     * @param nombre 
     * @return vector<int> 
     */
    vector<int> readVInt(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        vector<int> vecInt;
        fsR[nombre] >> vecInt;
        fsR.release();
        return vecInt;
    }
    /**
     * @brief  Lee un  vector de floats del fs con el nombre ingresado
     * 
     * @param nombre 
     * @return vector<float> 
     */
    vector<float> readVFloat(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        vector<float> vecFloat;
        fsR[nombre] >> vecFloat;
        fsR.release();
        return vecFloat;
    }

    /**
     * @brief Verifica si existe un entero con ese nombre en el fs
     * 
     * @param nombre 
     * @return true 
     * @return false 
     */
    bool existeInt(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        if((int)fsR[nombre]){
            fsR.release();
            return true;
        }
        fsR.release();
        return false;
    }
    /**
     * @brief Verifica si existe un flotante con ese nombre en el fs
     * 
     * @param nombre 
     * @return true 
     * @return false 
     */
    bool existeFloat(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        if((float)fsR[nombre]){
            fsR.release();
            return true;
        }
        fsR.release();
        return false;
    }
    /**
     * @brief Verifica si existe un string con ese nombre en el fs
     * 
     * @param nombre 
     * @return true 
     * @return false 
     */
    bool existeString(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        
        if( !((string)fsR[nombre]).empty() ){
            fsR.release();
            return true;
        }
        fsR.release();
        return false;
    }
    /**
     * @brief Verifica si existe un vector de enteros con ese nombre en el fs
     * 
     * @param nombre 
     * @return true 
     * @return false 
     */
    bool existeVInt(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        vector<int> vecInt;
        fsR[nombre] >> vecInt;
        if( ! vecInt.empty() ){
            fsR.release();
            return true;
        }
        fsR.release();
        return false;
    }
    /**
     * @brief Verifica si existe un vector de flotantes con ese nombre en el fs
     * 
     * @param nombre 
     * @return true 
     * @return false 
     */
    bool existeVFloat(string nombre){
        FileStorage fsR(dir, FileStorage::READ);
        vector<float> vecFloat;
        fsR[nombre] >> vecFloat;
        if( ! vecFloat.empty() ){
            fsR.release();
            return true;
        }
        fsR.release();
        return false;
    }



};


#endif