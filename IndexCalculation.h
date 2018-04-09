#ifndef INDEXCALCULATION_H
#define INDEXCALCULATION_H

#include "CommonFunctions.h"
#include "Color.h"
#include "Segmentation.h"
#include "Stitcher.h"

/**
 * @brief Calcula los indices de vegetación
 * 
 */
class IndexCalculation{
public:

	bool outputStitching;
	bool parallel;
	bool multispectral;
	bool sobreescribir;

	IndexCalculation(bool outputStitching = false,
		bool parallel = true,
		bool sobreescribir = true){
			this->outputStitching = outputStitching;
			this->parallel = parallel;
			this->sobreescribir = sobreescribir;
		}

	/**
	 * @brief Maneja todo el proceso para generación de los indices.
	 * 
	 */
	void processManager(){
		struct timeval beginAll;
		gettimeofday(&beginAll, NULL);


		vector<string> strImgs = obtenerInput(multispectral , outputStitching);

		if(parallel){
			cout << CommonFunctions::stringAzul("Con calculo paralelo \n");
			parallel_for_(Range(0, strImgs.size()), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					string strImg = CommonFunctions::removerExtension(CommonFunctions::obtenerUltimoDirectorio2(strImgs[i]));
					FSManager fs(strImg,"imagen");
					multispectral = (fs.readString("multiespectral")=="ms"? true:false);
					indexCalcu(strImgs[i],multispectral);
				}
			});
		}else{
			cout << CommonFunctions::stringAzul("Sin calculo paralelo \n");
			for(int i = 0;i < strImgs.size() ; i++){	
				string strImg = CommonFunctions::removerExtension(CommonFunctions::obtenerUltimoDirectorio2(strImgs[i]));
				FSManager fs(strImg,"imagen");
				multispectral = (fs.readString("multiespectral")=="ms"? true:false);
				indexCalcu(strImgs[i],multispectral);
			}
		}
		CommonFunctions::tiempo(beginAll, "Terminar todo: ");
	}
	/**
	 * @brief Decide que indices se van a calcular en base al tipo de imágenes que se procesen.
	 * 
	 * @param strImg 
	 * @param multispectral 
	 */
	void indexCalcu(string strImg, bool multispectral){
		struct timeval begin;
		gettimeofday(&begin, NULL);
		if(multispectral){
			if(sobreescribir || CommonFunctions::existFile(obtenerMSInputStr()+strImg)){
				cout << CommonFunctions::stringAzul("Comenzando calculo de indices para " + strImg + " multi-espectral \n");
				indexCalcuMS(strImg);
				CommonFunctions::tiempo(begin, "Terminar " + CommonFunctions::obtenerUltimoDirectorio(strImg)+":");
			}
		}else{
			if(sobreescribir || CommonFunctions::existFile(obtenerRGBInputStr()+strImg)){
				cout << CommonFunctions::stringAzul("Comenzando calculo de indices para " + strImg + " RGB \n");
				indexCalcuRGB(strImg);
				CommonFunctions::tiempo(begin, "Terminar " + CommonFunctions::obtenerUltimoDirectorio(strImg)+":");
			}
		}
	}
	/**
	 * @brief Calcula los indices para imágenes RGB.
	 * 
	 * @param strImg 
	 */
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
		ngrdiCalculation(BGRA,strImg);
		Mat separado = Segmentation::separarSuelo(BGRA);
		CommonFunctions::escribirImagen("Imagenes/Indices/output/rgb/"+ strImg +"/"+ strImg +"/suelo.png", separado);
	}
	/**
	 * @brief Calcula y escribe el indice RG=Rojo/Verde.
	 * 
	 * @param BGRA 
	 * @param strImg 
	 */
	void rgCalculation(vector<Mat> BGRA, string strImg){
		Mat rg,numerador,denominador;
		divide(BGRA[2],BGRA[1],rg,123,CV_8U);

		GaussianBlur( rg, rg, Size( 7,7 ), 0, 0 );

		escribirSegmentaciones(rg, BGRA[3],  strImg, "rg");
	}
	/**
	 * @brief Calcula y escribe el indice NGRDI
	 * 
	 * @param BGRA 
	 * @param strImg 
	 */
	void ngrdiCalculation(vector<Mat> BGRA, string strImg){
		Mat ngrdi,numerador,denominador;
		subtract(BGRA[1],BGRA[2],numerador,BGRA[3],CV_8S);
		add(BGRA[1],BGRA[2],denominador,BGRA[3],CV_8S);
		divide(numerador,denominador,ngrdi,1.,CV_32F);
		//CONVIERTO DE SIGNED A UCHAR
		
		ngrdi = (ngrdi + 1) * 128;
		ngrdi.convertTo(ngrdi,CV_8U);
		
		GaussianBlur( ngrdi, ngrdi, Size( 7,7 ), 0, 0 );
		
		escribirSegmentaciones(ngrdi, BGRA[3],  strImg, "ngrdi");
	}
	/**
	 * @brief Calcula los indices para imágenes multi-espectrales.
	 * 
	 * @param strImg 
	 */
	void indexCalcuMS(string strImg){
		Mat imgaux = imread(strImg, IMREAD_UNCHANGED);
		
		size_t position = strImg.find_last_of("/");
		strImg.erase(strImg.begin(),strImg.begin()+position+1);
		position = strImg.find_last_of(".");
		strImg.erase(strImg.begin()+position,strImg.end());

		vector<Mat> BGRA;
		split(imgaux, BGRA);
		//CORRIJO EL PROBLEMA DE QUE EL INFRAROJO 'INVADE' EL ROJO
		subtract(BGRA[2],BGRA[0],BGRA[2],cv::noArray(),CV_8U);
		if(BGRA.size() == 3){
			BGRA.push_back(Mat(BGRA[0].size(),CV_8U,Scalar(255,255,255)));
		}
		//CALCULO NDVI
		ndviCalculation(BGRA, strImg);
		// //CALCULO RVI
		rviCalculation(BGRA, strImg);
	}

	/**
	 * @brief Calcula y escribe el indice NDVI=(Infrarojo-Rojo)/(Infrarojo+Rojo).
	 * 
	 * @param BGRA 
	 * @param strImg 
	 */
	void ndviCalculation(vector<Mat> BGRA, string strImg){
		Mat ndvi,numerador,denominador;
		//OBTENGO EL NDVI
		subtract(BGRA[0],BGRA[2],numerador,BGRA[3],CV_8S);
		add(BGRA[0],BGRA[2],denominador,BGRA[3],CV_8S);
		divide(numerador,denominador,ndvi,1.,CV_32F);
		//CONVIERTO DE SIGNED A UCHAR
		
		ndvi = (ndvi + 1) * 128;
		ndvi.convertTo(ndvi,CV_8U);

		GaussianBlur( ndvi, ndvi, Size( 7,7 ), 0, 0 );
		
		escribirSegmentaciones(ndvi, BGRA[3],  strImg, "ndvi");
	}
	/**
	 * @brief Calcula y escribe el indice RVI=Infrarojo/Rojo
	 * 
	 * @param BGRA 
	 * @param strImg 
	 */
	void rviCalculation(vector<Mat> BGRA, string strImg){
		Mat rvi,numerador,denominador;
		//OBTENGO EL NDVI
		divide(BGRA[0],BGRA[2],rvi,127,CV_8U);

		GaussianBlur( rvi, rvi, Size( 7,7 ), 0, 0 );
		
		escribirSegmentaciones(rvi, BGRA[3], strImg, "rvi");
	}
	/**
	 * @brief  Escribe los resultados de un indice con diferentes segmentaciones.
	 * 
	 * @param indice 
	 * @param trans 
	 * @param strImg 
	 * @param strIndex 
	 */
	void escribirSegmentaciones(Mat indice, Mat trans, string strImg,string strIndex){
		String Nombre;
		if(strIndex != "rvi" && strIndex != "ndvi"){
			Nombre = "Imagenes/Indices/output/rgb/"+ strImg + "/" + strIndex+ "/";
		}else{
			Nombre = "Imagenes/Indices/output/ms/"+ strImg + "/" + strIndex+ "/";
		}
		Mat indiceCuantizado = Segmentation::segmentationVariation(indice,trans,5);
		Mat indiceLut = Segmentation::createLut(indice,trans);
		vector<Mat> indiceChart = Segmentation::generarGrafico(indice, 20,trans,strImg,strIndex);
		CommonFunctions::escribirImagen(Nombre +"Index.png", CommonFunctions::addAlpha(indice,trans) );
		CommonFunctions::escribirImagen(Nombre +"Cuantizado.png", indiceCuantizado );
		CommonFunctions::escribirImagen(Nombre +"Lut.png", indiceLut );
		CommonFunctions::escribirImagen(Nombre +"Chart.png", indiceChart[0] );
		CommonFunctions::escribirImagen(Nombre +"ChartImg.png", CommonFunctions::addAlpha(indiceChart[1],trans) );
	}
	/**
	 * @brief Obtiene las ubicaciones de las imágenes de entrada en base al tipo de imágen y a la BD que la contiene.
	 * 
	 * @param multiespectral 
	 * @param outputStitching 
	 * @return vector<string> 
	 */
	vector<string> obtenerInput(bool multiespectral, bool outputStitching){
		if(outputStitching){
			uav::Stitcher stitch;
			vector<string> str = CommonFunctions::obtenerImagenes(stitch.obtenerOutputRF().c_str());
			return str;
		}
		if(multiespectral){
			return obtenerMSInput();
		}
		return obtenerRGBInput();
	}
	/**
	 * @brief Obtiene las ubicaciones de las imágenes RGB de entrada.
	 * 
	 * @return vector<string> 
	 */
	vector<string> obtenerRGBInput(){
		return CommonFunctions::obtenerImagenes(obtenerRGBInputStr().c_str());
	}

	string obtenerRGBInputStr(){
		return "Imagenes/Indices/input/rgb/";
	}
	/**
	 * @brief Obtiene las ubicaciones de las imágenes multi-espectrales de entrada.
	 * 
	 * @return vector<string> 
	 */
	vector<string> obtenerMSInput(){
		return CommonFunctions::obtenerImagenes(obtenerMSInputStr().c_str());;
	}

	string obtenerMSInputStr(){
		return "Imagenes/Indices/input/ms/";
	}
	/**
	 * @brief Obtiene las ubicaciones de las imágenes RGB de salida.
	 * 
	 * @return vector<string> 
	 */
	vector<string> obtenerRGBOutput(){
		return CommonFunctions::obtenerImagenes("Imagenes/Indices/output/rgb/");;
	}
	/**
	 * @brief  Obtiene las ubicaciones de las imágenes multi-espectrales de salida.
	 * 
	 * @return vector<string> 
	 */
	vector<string> obtenerMSOutput(){
		return CommonFunctions::obtenerImagenes("Imagenes/Indices/output/ms/");;
	}
	string obtenerIndex(string strImg, string strIndex){
		if(strIndex == "ndvi" || strIndex == "rvi"){
			return "Imagenes/Indices/output/ms/"+strImg+"/"+strIndex+"/Index.png";
		}else{
			return "Imagenes/Indices/output/rgb/"+strImg+"/"+strIndex+"/Index.png";
		}
	}
	string obtenerCuant(string strImg, string strIndex){
		if(strIndex == "ndvi" || strIndex == "rvi"){
			return "Imagenes/Indices/output/ms/"+strImg+"/"+strIndex+"/Cuantizado.png";
		}else{
			return "Imagenes/Indices/output/rgb/"+strImg+"/"+strIndex+"/Cuantizado.png";
		}
	}
	string obtenerChart(string strImg, string strIndex){
		if(strIndex == "ndvi" || strIndex == "rvi"){
			return "Imagenes/Indices/output/ms/"+strImg+"/"+strIndex+"/Chart.png";
		}else{
			return "Imagenes/Indices/output/rgb/"+strImg+"/"+strIndex+"/Chart.png";
		}
	}
	string obtenerChartImg(string strImg, string strIndex){
		if(strIndex == "ndvi" || strIndex == "rvi"){
			return "Imagenes/Indices/output/ms/"+strImg+"/"+strIndex+"/ChartImg.png";
		}else{
			return "Imagenes/Indices/output/rgb/"+strImg+"/"+strIndex+"/ChartImg.png";
		}
	}
	string obtenerLut(string strImg, string strIndex){
		if(strIndex == "ndvi" || strIndex == "rvi"){
			return "Imagenes/Indices/output/ms/"+strImg+"/"+strIndex+"/Lut.png";
		}else{
			return "Imagenes/Indices/output/rgb/"+strImg+"/"+strIndex+"/Lut.png";
		}
	}

};


#endif