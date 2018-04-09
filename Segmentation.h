#ifndef Segmentation_H
#define Segmentation_H

#include "CommonFunctions.h"
#include "Color.h"
#include "FSManager.h"

/**
 * @brief Genera las diferentes segmentaciones.
 * 
 */
class Segmentation{
public:
    double min=0;
	double max=0;
	/**
	 * @brief Realiza una normalizacion de la imagen (gris) y luego cuantiza sus valores para reducir las escalas de grises
	 * 
	 * @param img 
	 * @param trans 
	 * @param cantColores 
	 * @return Mat 
	 */
	Mat static segmentationVariation(Mat img, Mat trans, int cantColores){
		//recupero fondo blanco
		Mat transAux = trans.clone();
        double min,max;
		//Normalizo la matriz, que seria como ecualizarla
		img = normalizateMat(img,transAux,min,max);
		int denominador = 256/cantColores;
		//convierto a flotante
		transAux.convertTo(transAux,CV_32F);
		img.convertTo(img,CV_32F);
		transAux/=256;
		img/=256;
		//aplico transparencia
		img = img.mul(transAux);
		//aplico la cuantizacion
		img.convertTo(img,CV_8U,256/denominador,.5);
		img*=denominador;
		return CommonFunctions::addAlpha(img,trans);
	}
	/**
	 * @brief Crea y aplica un mapa de colores a una imágen y luego genera y pega un indice para que se pueda ver que valor tiene cada color en una imagen.
	 * 
	 * @param temp 
	 * @param trans 
	 * @return Mat 
	 */
	Mat static  createLut(Mat temp, Mat trans){
        double min,max;
		//Normalizo la matriz, que seria como ecualizarla
		temp = normalizateMat(temp, trans,min,max);
		// creo las matrices que mapean el valor del pixel rx (1-256), con el valor que va a tener cada canal (BGR) Mx
		Mat M1(1,256,CV_8U);
		Mat M2(1,256,CV_8U);
		Mat M3(1,256,CV_8U);
		for(int i=0;i<256;i++)
		{
			M1.at<uchar>(i)=0;
			if(i<129){
				M2.at<uchar>(i)=2*i > 255?255:2*i;
				M3.at<uchar>(i)=255;
			}else{
				M2.at<uchar>(i)=M2.at<uchar>(i-1)-1;
				M3.at<uchar>(i)=M3.at<uchar>(i-1)-2;
			}
		}
	
		Mat r1,r2,r3;
		// aplico el mapeo de Mx-rx
		cv::LUT(temp,M1,r1);
		cv::LUT(temp,M2,r2);
		cv::LUT(temp,M3,r3);
		std::vector<cv::Mat> planes;
		planes.push_back(r1);
		planes.push_back(r2);
		planes.push_back(r3);
		Mat dst;
		cv::merge(planes,dst);
		//Agrego el cartel que muestra como es el LUT
		std::vector<cv::Mat> planes2;
		planes2.push_back(M1);
		planes2.push_back(M2);
		planes2.push_back(M3);
		Mat auxDst;
		cv::merge(planes2,auxDst);
		dst = drawIndexOfIndex(dst,auxDst,trans,min,max);

		return dst;
	}
	/**
	 * @brief Genera y pega un indice para que se pueda ver que valor tiene cada color en una imagen.
	 * 
	 * @param img 
	 * @param Lut 
	 * @param trans 
	 * @param min 
	 * @param max 
	 * @return Mat 
	 */
	Mat static  drawIndexOfIndex(Mat img,Mat Lut,Mat trans,double min, double max){
		//creo imagen blanca
		int sizeIndex = img.cols > img.rows? img.cols : img.rows;
		int resize = 30;
		Mat index(Size(sizeIndex/resize,sizeIndex/(resize/2)),CV_8UC3,Scalar(255,255,255));
		//agrego el cuadro a la transparencia
		Mat transAux = trans.clone();
		Mat indexTrans(Size(sizeIndex/resize,sizeIndex/(resize/2)),CV_8U,255);
		indexTrans.copyTo(transAux(Rect(trans.cols-1.1*indexTrans.cols,trans.rows-1.1*indexTrans.rows,indexTrans.cols,indexTrans.rows)));
		
		//copio pixel a pixel el LUT a el index
		for(int i = 0 ; i < index.rows ; i++){
			for(int j = 0; j < index.cols;j++){
				int iNorm = i * Lut.cols / index.rows;
				// int jNorm = j * Lut.cols / index.cols;
				index.at<Vec3b>(i,j)[0] = Lut.at<Vec3b>(0,iNorm)[0];
				index.at<Vec3b>(i,j)[1] = Lut.at<Vec3b>(0,iNorm)[1];
				index.at<Vec3b>(i,j)[2] = Lut.at<Vec3b>(0,iNorm)[2];
			}
		}
		//ingreso el texto
		float auxMin = ((min>0)?min:0)/256;
		float auxMax = ((max<256)?max:255)/256;
		putText(index, CommonFunctions::fToS(auxMin), cvPoint(index.cols*.1,index.rows*.2), 
    	FONT_HERSHEY_TRIPLEX, (float)index.rows/170, cvScalar(0,0,0), (float)index.rows/170*3, CV_AA);
		putText(index, CommonFunctions::fToS(auxMin+(auxMax-auxMin)/2), cvPoint(index.cols*.1,index.rows*.55), 
    	FONT_HERSHEY_TRIPLEX, (float)index.rows/200, cvScalar(0,0,0),(float)index.rows/200*3, CV_AA);
		putText(index, CommonFunctions::fToS(auxMax), cvPoint(index.cols*.1,index.rows*.9), 
    	FONT_HERSHEY_TRIPLEX, (float)index.rows/170, cvScalar(0,0,0), (float)index.rows/170*3, CV_AA);
		//copio el indice a la imagen
		index.copyTo(img(Rect(img.cols-1.1*index.cols,img.rows-1.1*index.rows,index.cols,index.rows)));

		img = CommonFunctions::addAlpha(img,transAux);
		return img;
	}
	/**
	 * @brief Normaliza los pixeles de una imágen.
	 * 
	 * @param img 
	 * @param mask 
	 * @param min 
	 * @param max 
	 * @return Mat 
	 */
	Mat static normalizateMat(Mat img, Mat mask,double &min, double &max){
		Mat std,mean,dst;
		//calculo minimo, maximo, media y desviacion estandar
		minMaxLoc(img,&min,&max,0,0,mask);
		meanStdDev(img, mean, std, mask);
		// CommonFunctions::histDraw(img,"asd1");
		// dst = img - (mean.at<double>(0,0) - std.at<double>(0,0));
		//pongo un minimo, un maximo y los uso para realizar la normalizacion
		min = mean.at<double>(0,0) - 4*std.at<double>(0,0);
		max = mean.at<double>(0,0) + 4*std.at<double>(0,0);
		dst = img - min;
		// CommonFunctions::histDraw(dst,"asd2");
		dst = dst * ( 256/(max-min) );
		// CommonFunctions::histDraw(dst,"asd3");
				
		return dst;
	}
	/**
	 * @brief Aplica una cuantización de una imágen y a cada cuantizado le asigna su porcentaje de aparición en la imágen, con esto se realiza un gráfico.
	 * 
	 * @param img 
	 * @param cantidad 
	 * @param trans 
	 * @return vector<Mat> 
	 */
	vector<Mat> static generarGrafico(Mat img,int cantidad, Mat trans,string strImg, string strIndex){
		Mat imgColor;
		cvtColor(img,imgColor,CV_GRAY2BGR);
		long int cantPix = 0;
		vector<int> limites;
		//en base a la cantdad de intervalos q quiere genero los limites
		for(int i = 0 ; i <= cantidad; i++){
			limites.push_back(i*256/cantidad);
		}
		vector<long int> contador(limites.size()-1,0);
		vector<Vec3f> colors = Color::generarColores();
		//cuento la cantidad de pixeles que esten en cada intervalo
		for(int i = 0 ; i < img.rows ; i++){
			for(int j = 0 ; j < img.cols ; j++){
				int pixVal = img.at<uchar>(i,j);
				if(trans.at<uchar>(i,j) == 255){
					cantPix++;
					for(int k = 1;k < limites.size(); k++){
						if(limites[k-1] <= pixVal && pixVal < limites[k]){
							contador[k-1]++;
							imgColor.at<Vec3b>(i,j) = colors[k-1]*256;
						}
					}
				}
			}
		}
		vector<float> porcentaje(limites.size()-1,0);
		//paso todo a porcentaje
		for(int k = 0;k < contador.size(); k++){
			porcentaje[k] = contador[k] / ((float)cantPix/100);
		}
		//generl el grafico con colores aleatorios
		Vec3f color;
		Mat chart(Size(1500,500),CV_32FC3,Scalar(0,0,0));
		float contadorAcum = 0;
		int posText = 2;
		float fontSize = 2.2; float thickness = 2; int xPos = 1020; int yPos = posText*35;
		putText(chart, "min  max  porcentaje" , cvPoint(xPos,yPos*0.4), FONT_HERSHEY_PLAIN, fontSize, cvScalar(1,1,1), thickness, CV_AA);
		for(int k = 0;k < porcentaje.size();k++){
			yPos = posText*35;
			if(porcentaje[k] > .3){
				//dibujo pixel a pixel
				Vec3f color = colors[k];
				Vec3f colorText = color;

				for(int i = chart.rows/2 ; i >= 0 ; i--){
					for(int j = contadorAcum*10; j < (contadorAcum + porcentaje[k])*10-1; j++){
						chart.at<Vec3f>(i,j) = color;
					}
					color[0] *= .995;color[1] *= .995;color[2] *= .995;
				}
				color = colorText;
				for(int i = chart.rows/2 ; i < chart.rows ; i++){
					for(int j = contadorAcum*10; j < (contadorAcum + porcentaje[k])*10-1; j++){
						chart.at<Vec3f>(i,j) = color;
					}
					color[0] *= .995;color[1] *= .995;color[2] *= .995;
				}
				//agrego el texto al grafico
				putText(chart, to_string(limites[k]) , cvPoint(xPos+10,yPos), FONT_HERSHEY_PLAIN, fontSize, cvScalar(255,255,255), thickness+1, CV_AA);
				putText(chart, to_string(limites[k]) , cvPoint(xPos+10,yPos), FONT_HERSHEY_PLAIN, fontSize, cvScalar(colorText[0],colorText[1],colorText[2]), thickness, CV_AA);
				putText(chart, to_string(limites[k+1]) , cvPoint(xPos+110,yPos), FONT_HERSHEY_PLAIN, fontSize, cvScalar(255,255,255), thickness+1, CV_AA);
				putText(chart, to_string(limites[k+1]) , cvPoint(xPos+110,yPos), FONT_HERSHEY_PLAIN, fontSize, cvScalar(colorText[0],colorText[1],colorText[2]), thickness, CV_AA);
				putText(chart, CommonFunctions::fToS(porcentaje[k],6) + "%" , cvPoint(xPos+230,yPos), FONT_HERSHEY_PLAIN, fontSize, cvScalar(255,255,255), thickness+1, CV_AA);
				putText(chart, CommonFunctions::fToS(porcentaje[k],6) + "%" , cvPoint(xPos+230,yPos), FONT_HERSHEY_PLAIN, fontSize, cvScalar(colorText[0],colorText[1],colorText[2]), thickness, CV_AA);
				posText++;
				contadorAcum += porcentaje[k];
			}
		}

		FSManager fs(strImg,"imagen");
		fs.appendVInt("limites"+strIndex,limites);
		fs.appendVFloat("porcentaje"+strIndex,porcentaje);
		
		chart*=256;
		chart.convertTo(chart,CV_8U);
		vector<Mat> vecMat;
		vecMat.push_back(chart);
		vecMat.push_back(imgColor);
		return vecMat;
	}
	/**
	 * @brief Separa el suelo de la vegetación.
	 * 
	 * @param BGRA 
	 * @return Mat 
	 */

    Mat static separarSuelo(vector<Mat> BGRA){
		Mat b,g,r;
		int min = 110;
		int max = 255;
		threshold(BGRA[0], b, min, max, THRESH_BINARY);
		threshold(BGRA[1], g, min, max, THRESH_BINARY);
		threshold(BGRA[2], r, min, max, THRESH_BINARY);
		Mat maskNoSuelo = (b+g+r);
		Mat resultado;
		Mat auxAlpha[4]={0*maskNoSuelo,(255-maskNoSuelo)/2,maskNoSuelo/2,BGRA[3]};
		merge(auxAlpha,4,resultado);

		return resultado;
	}

};

#endif