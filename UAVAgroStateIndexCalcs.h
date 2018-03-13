#ifndef UAVAGROSTATEINDEXCALCS_H
#define UAVAGROSTATEINDEXCALCS_h

#include "CommonFunctions.h"

class UAVAgroStateIndexCalcs
{
public:

	
	Mat static indexCalcu(string strImg){
		Mat imgaux = imread(strImg, IMREAD_UNCHANGED);
		
		size_t position = strImg.find_last_of("/");
		strImg.erase(strImg.begin(),strImg.begin()+position);
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
		Mat ndvi = ndviCalculation(BGRA);
		Mat ndviCuantizado = segmentationVariation(ndvi,BGRA[3],5);
		imwrite("Imagenes/NDVI/output/"+ strImg +"-ndvi.png", addAlpha(ndvi,BGRA[3]) );
		imwrite("Imagenes/NDVI/output/"+ strImg +"-ndviCuantizado.png", addAlpha(ndviCuantizado,BGRA[3]) );
		//CALCULO RVI
		Mat rvi = rviCalculation(BGRA);
		Mat rviCuantizado = segmentationVariation(rvi,BGRA[3],5);
		imwrite("Imagenes/NDVI/output/"+ strImg +"-rvi.png", addAlpha(rvi,BGRA[3]) );
		imwrite("Imagenes/NDVI/output/"+ strImg +"-rviCuantizado.png", addAlpha(rviCuantizado,BGRA[3]) );


	
		return ndvi;
	}

	Mat static ndviCalculation(vector<Mat> BGRA){
		Mat division,numerador,denominador;
		//OBTENGO EL NDVI
		subtract(BGRA[0],BGRA[2],numerador,BGRA[3],CV_8S);
		add(BGRA[0],BGRA[2],denominador,BGRA[3],CV_8S);
		divide(numerador,denominador,division,1.,CV_32F);
		//CONVIERTO DE SIGNED A UCHAR

		division = (division * 127) + 127;
		division.convertTo(division,CV_8U);

		return division;
	}
	Mat static rviCalculation(vector<Mat> BGRA){
		Mat division,numerador,denominador;
		//OBTENGO EL NDVI
		BGRA[0].convertTo(numerador,CV_8S);
		BGRA[2].convertTo(denominador,CV_8S);
		divide(numerador,denominador,division,1.,CV_32F);
		//CONVIERTO DE SIGNED A UCHAR
		division = (division + 1) * 128;
		division.convertTo(division,CV_8U);
		
		return division;
	}

	Mat static addAlpha(Mat img, Mat trans){
		Mat resultado;
		if(!trans.empty()){
			Mat auxAlpha[4]={img,img,img,trans};
			merge(auxAlpha,4,resultado);
		}else{
			return img;
		}
		return resultado;
	}

	Mat static segmentationVariation(Mat img, Mat trans, int cantColores){
		//recupero fondo blanco
		img = normalizateMat(img,trans);
		int denominador = 256/cantColores;
		trans.convertTo(trans,CV_32F);
		img.convertTo(img,CV_32F);
		trans/=256;
		img/=256;
		img = img.mul(trans);
		img.convertTo(img,CV_8U,256/denominador,.5);
		img*=denominador;

		return img;
	}

	Mat static normalizateMat(Mat img, Mat mask){
		Mat std,mean,dst;
		double max,min;
		meanStdDev(img, mean, std, mask);
		
		dst = img - (mean.at<double>(0,0) - 1.5 * std.at<double>(0,0));
		minMaxLoc(dst, &min, &max,0,0,mask);
		// cout<< max<<endl;
		dst = dst * (256/max);
		
		return dst;
	}

	Mat static createLut(Mat temp){
		double min,max;
		minMaxLoc(temp, &min, &max);
		Mat M1(1,256,CV_8U);
		Mat M2(1,256,CV_8U);
		Mat M3(1,256,CV_8U);
		for(int i=min;i<max;i++)
		{
			M1.at<uchar>(i)=0;
			M2.at<uchar>(i)=(i-min)*256/(max-min);
			M3.at<uchar>(i)=255 - (i-min)*256/(max-min);
		}
	
		Mat r1,r2,r3;
		
		cv::LUT(temp,M1,r1);
		cv::LUT(temp,M2,r2);
		cv::LUT(temp,M3,r3);
		std::vector<cv::Mat> planes;
		planes.push_back(r1);
		planes.push_back(r2);
		planes.push_back(r3);
		Mat dst;
		cv::merge(planes,dst);

		return dst;
	}

private:
    
};


#endif