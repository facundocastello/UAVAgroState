#ifndef UAVAGROSTATEINDEXCALCS_H
#define UAVAGROSTATEINDEXCALCS_h

#include "CommonFunctions.h"

class UAVAgroStateIndexCalcs
{
public:

	Mat static ndviCalcu(string strImg){
		Mat imgaux = imread(strImg, IMREAD_UNCHANGED);
		return ndviCalcu(imgaux);;
	}

	Mat static ndviCalcu(Mat imgaux){
		
		// size_t position = strImg.find_last_of("/");
		// strImg.erase(strImg.begin(),strImg.begin()+position);

		imwrite("Imagenes/resultados/NDVI/original.png", imgaux);
		
		vector<Mat> BGRA;
		split(imgaux, BGRA);	
		Mat numerador,denominador,division;
		
		subtract(BGRA[0],BGRA[2],numerador,cv::noArray(),CV_8S);
		add(BGRA[0],BGRA[2],denominador,cv::noArray(),CV_8S);
		divide(numerador,denominador,division,1.,CV_32F);
		
		division = (division * 127) + 127;
		division.convertTo(division,CV_8U);
		Mat resultadogris = division;
		// cout<<BGRA[3].empty()<<endl;
		if(!BGRA[3].empty()){
			Mat auxAlpha[4]={division,division,division,BGRA[3]};
			merge(auxAlpha,4,resultadogris);
		};
		imwrite("Imagenes/resultados/NDVI/resultadogris.png", resultadogris);

		Mat resultadosalida;
		Mat auxSalida[3]={division,division,division};
		merge(auxSalida,3,resultadosalida);

		applyColorMap(division, division, COLORMAP_JET);
		vector<Mat> BGR;
		split(division,BGR);
		Mat resultadocolor = division;
		if(!BGRA[3].empty()){
			Mat auxAlpha2[4]={BGR[0],BGR[1],BGR[2],BGRA[3]};
			merge(auxAlpha2,4,resultadocolor);
		}
		imwrite("Imagenes/resultados/NDVI/resultadocolor.png", resultadocolor);
	
		return resultadosalida;
	}

private:
    
};


#endif