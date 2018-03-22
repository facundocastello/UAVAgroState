#ifndef INDEXCALCULATION_H
#define INDEXCALCULATION_H

#include "CommonFunctions.h"
#include "Color.h"
#include "Segmentation.h"


class IndexCalculation{
public:


	void processManager(){
		
		struct timeval beginAll;
		struct timeval begin;
		gettimeofday(&beginAll, NULL);
		gettimeofday(&begin, NULL);
		bool parallel = true;
		bool multispectral = true;
		if(multispectral){
			vector<string> strNDVI = CommonFunctions::obtenerImagenes("Imagenes/Indices/input/ms/");
			cout << "Comenzando calculo de indices para ";
			cout<< "multi-espectral ";
			if(parallel){
				cout << "con paralelismo \n";
				parallel_for_(Range(0, strNDVI.size()), [&](const Range& range){
					for(int i = range.start;i < range.end ; i++){
					indexCalcuMS(strNDVI[i]);
					CommonFunctions::tiempo(begin, "Terminar " + CommonFunctions::obtenerUltimoDirectorio(strNDVI[i])+":");
					}
				});
			}else{
				cout << "sin paralelismo \n";
				for(int i = 0;i < strNDVI.size() ; i++){
					indexCalcuMS(strNDVI[i]);
					begin = CommonFunctions::tiempo(begin, "Terminar " + CommonFunctions::obtenerUltimoDirectorio(strNDVI[i])+":");
				}
			}
		}else{
			cout << "Comenzando calculo de indices para ";
			cout << "rgb \n";
			vector<string> strNDVI = CommonFunctions::obtenerImagenes("Imagenes/Indices/input/rgb/");
			if(parallel){
				cout << "con paralelismo \n";
				parallel_for_(Range(0, strNDVI.size()), [&](const Range& range){
					for(int i = range.start;i < range.end ; i++){
					indexCalcuRGB(strNDVI[i]);
					CommonFunctions::tiempo(begin, "Terminar " + CommonFunctions::obtenerUltimoDirectorio(strNDVI[i])+":");
					}
				});
			}else{
				cout << "sin paralelismo \n";
				for(int i = 0;i < strNDVI.size() ; i++){
					indexCalcuRGB(strNDVI[i]);
					begin = CommonFunctions::tiempo(begin, "Terminar " + CommonFunctions::obtenerUltimoDirectorio(strNDVI[i])+":");
				}
			}
		}
		CommonFunctions::tiempo(beginAll, "Terminar todo: ");
	}

	void indexCalcuRGB(string strImg){
		Mat imgaux = imread(strImg, IMREAD_UNCHANGED);
		
		size_t position = strImg.find_last_of("/");
		strImg.erase(strImg.begin(),strImg.begin()+position+1);
		position = strImg.find_last_of(".");
		strImg.erase(strImg.begin()+position,strImg.end());

		vector<Mat> BGRA;
		split(imgaux, BGRA);
		//CORRIJO EL PROBLEMA DE QUE EL INFRAROJO 'INVADE' EL ROJO
		// subtract(BGRA[2],BGRA[0]*0.8,BGRA[2],cv::noArray(),CV_8U);
		if(BGRA.size() == 3){
			BGRA.push_back(Mat(BGRA[0].size(),CV_8U,Scalar(255,255,255)));
		}

		rgCalculation(BGRA,strImg);
		Mat separado = Segmentation::separarSuelo(BGRA);
		CommonFunctions::escribirImagen("Imagenes/Indices/output/rgb/"+ strImg +"/"+ strImg +"suelo.png", separado);
	}



	void rgCalculation(vector<Mat> BGRA, string strImg){
		Mat rg,numerador,denominador;
		divide(BGRA[2],BGRA[1],rg,123,CV_8U);
		
		String escribir = "Imagenes/Indices/output/rgb/"+ strImg +"/rg/"+ strImg +"rg";
		escribirSegmentaciones(rg, BGRA[3], escribir);
	}

	void indexCalcuMS(string strImg){
		Mat imgaux = imread(strImg, IMREAD_UNCHANGED);
		
		size_t position = strImg.find_last_of("/");
		strImg.erase(strImg.begin(),strImg.begin()+position+1);
		position = strImg.find_last_of(".");
		strImg.erase(strImg.begin()+position,strImg.end());

		vector<Mat> BGRA;
		split(imgaux, BGRA);
		//CORRIJO EL PROBLEMA DE QUE EL INFRAROJO 'INVADE' EL ROJO
		subtract(BGRA[2],BGRA[0]*0.8,BGRA[2],cv::noArray(),CV_8U);
		if(BGRA.size() == 3){
			BGRA.push_back(Mat(BGRA[0].size(),CV_8U,Scalar(255,255,255)));
		}
		//CALCULO NDVI
		ndviCalculation(BGRA, strImg);
		// //CALCULO RVI
		rviCalculation(BGRA, strImg);
	}


	void ndviCalculation(vector<Mat> BGRA, string strImg){
		Mat ndvi,numerador,denominador;
		//OBTENGO EL NDVI
		subtract(BGRA[0],BGRA[2],numerador,BGRA[3],CV_8S);
		add(BGRA[0],BGRA[2],denominador,BGRA[3],CV_8S);
		divide(numerador,denominador,ndvi,1.,CV_32F);
		//CONVIERTO DE SIGNED A UCHAR
		
		ndvi = (ndvi + 1) * 128;
		ndvi.convertTo(ndvi,CV_8U);
		
		String escribir = "Imagenes/Indices/output/ms/"+ strImg +"/ndvi/"+ strImg + "ndvi";
		escribirSegmentaciones(ndvi, BGRA[3], escribir);
	}
	void rviCalculation(vector<Mat> BGRA, string strImg){
		Mat rvi,numerador,denominador;
		//OBTENGO EL NDVI
		divide(BGRA[0],BGRA[2],rvi,127,CV_8U);

		String escribir = "Imagenes/Indices/output/ms/"+ strImg +"/rvi/"+ strImg + "rvi";
		escribirSegmentaciones(rvi, BGRA[3], escribir);
	}

	void escribirSegmentaciones(Mat indice, Mat trans, string Nombre){
		Mat indiceCuantizado = Segmentation::segmentationVariation(indice,trans,5);
		Mat indiceLut = Segmentation::createLut(indice,trans);
		vector<Mat> indiceChart = Segmentation::generarGrafico(indice, 20,trans);
		CommonFunctions::escribirImagen(Nombre +".png", Segmentation::addAlpha(indice,trans) );
		CommonFunctions::escribirImagen(Nombre +"Cuantizado.png", indiceCuantizado );
		CommonFunctions::escribirImagen(Nombre +"Lut.png", indiceLut );
		CommonFunctions::escribirImagen(Nombre +"Chart.png", indiceChart[0] );
		CommonFunctions::escribirImagen(Nombre +"ChartImg.png", Segmentation::addAlpha(indiceChart[1],trans) );
	}

};


#endif