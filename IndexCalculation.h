#ifndef INDEXCALCULATION_H
#define INDEXCALCULATION_H

#include "CommonFunctions.h"
#include "Color.h"
#include "Segmentation.h"
#include "Stitcher.h"


class IndexCalculation{
public:


	void processManager(){
		
		struct timeval beginAll;
		struct timeval begin;
		gettimeofday(&beginAll, NULL);
		gettimeofday(&begin, NULL);
		bool outputStitching = false;
		bool parallel = true;
		bool multispectral = true;
		vector<string> strImgs = obtenerInput(multispectral , outputStitching);
		if(parallel){
			cout << "con paralelismo \n";
			parallel_for_(Range(0, strImgs.size()), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
				indexCalcu(strImgs[i],multispectral);
				CommonFunctions::tiempo(begin, "Terminar " + CommonFunctions::obtenerUltimoDirectorio(strImgs[i])+":");
				}
			});
		}else{
			cout << "sin paralelismo \n";
			for(int i = 0;i < strImgs.size() ; i++){
				indexCalcu(strImgs[i],multispectral);
				begin = CommonFunctions::tiempo(begin, "Terminar " + CommonFunctions::obtenerUltimoDirectorio(strImgs[i])+":");
			}
		}
		CommonFunctions::tiempo(beginAll, "Terminar todo: ");
	}

	void indexCalcu(string strImg, bool multispectral){
		if(multispectral){
			cout << "Comenzando calculo de indices para " + strImg + " multi-espectral \n";
			indexCalcuMS(strImg);
		}else{
			cout << "Comenzando calculo de indices para " + strImg + " RGB \n";
			indexCalcuRGB(strImg);
		}
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
		CommonFunctions::escribirImagen("Imagenes/Indices/output/rgb/"+ strImg +"/"+ strImg +"suelo.../", separado);
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
		CommonFunctions::escribirImagen(Nombre +".png", CommonFunctions::addAlpha(indice,trans) );
		CommonFunctions::escribirImagen(Nombre +"Cuantizado.png", indiceCuantizado );
		CommonFunctions::escribirImagen(Nombre +"Lut.png", indiceLut );
		CommonFunctions::escribirImagen(Nombre +"Chart.png", indiceChart[0] );
		CommonFunctions::escribirImagen(Nombre +"ChartImg.png", CommonFunctions::addAlpha(indiceChart[1],trans) );
	}

	vector<string> obtenerInput(bool multiespectral, bool outputStitching){
		if(outputStitching){
			uav::Stitcher stitch;
			vector<string> str;
			str.push_back(stitch.obtenerOutputRF());
			return str;
		}
		if(multiespectral){
			return obtenerMSInput();
		}
		return obtenerRGBInput();
	}

	vector<string> obtenerRGBInput(){
		return CommonFunctions::obtenerImagenes("Imagenes/Indices/input/rgb/");;
	}

	vector<string> obtenerMSInput(){
		return CommonFunctions::obtenerImagenes("Imagenes/Indices/input/ms/");;
	}

	vector<string> obtenerRGBOutput(){
		return CommonFunctions::obtenerImagenes("Imagenes/Indices/output/rgb/");;
	}

	vector<string> obtenerMSOutput(){
		return CommonFunctions::obtenerImagenes("Imagenes/Indices/output/ms/");;
	}

};


#endif