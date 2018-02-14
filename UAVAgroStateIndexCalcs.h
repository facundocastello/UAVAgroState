#ifndef UAVAGROSTATEINDEXCALCS_H
#define UAVAGROSTATEINDEXCALCS_h

#include "CommonFunctions.h"

class UAVAgroStateIndexCalcs
{
public:

	
	Mat static ndviCalcu(string strImg){
		Mat imgaux = imread(strImg, IMREAD_UNCHANGED);
		
		size_t position = strImg.find_last_of("/");
		strImg.erase(strImg.begin(),strImg.begin()+position);

		imwrite("Imagenes/NDVI/output/"+ strImg +"original.png", imgaux);

		vector<Mat> BGRA;
		split(imgaux, BGRA);	
		Mat numerador,denominador,division;
		Mat newred;

		subtract(BGRA[2],BGRA[0]*0.9,BGRA[2],cv::noArray(),CV_8U);
		merge(BGRA,newred);
		imwrite("Imagenes/NDVI/output/"+ strImg +"newred.png", newred);
		
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
		imwrite("Imagenes/NDVI/output/"+ strImg +"resultadogris.png", resultadogris);

		Mat resultadosalida;
		Mat auxSalida[3]={division,division,division};
		merge(auxSalida,3,resultadosalida);


		double min,max;
		minMaxLoc(division, &min, &max);
		Mat resultadoNormalizado = (division - min) * 256 / (max-min);
		applyColorMap(resultadoNormalizado, resultadoNormalizado, COLORMAP_JET);
		
		division = createLut(division);
		// applyColorMap(division, division, COLORMAP_JET);
		Mat resultadocolor = division;
		if(!BGRA[3].empty()){
			vector<Mat> BGR;
			split(division,BGR);
			Mat auxAlpha2[4]={BGR[0],BGR[1],BGR[2],BGRA[3]};
			merge(auxAlpha2,4,resultadocolor);
			split(resultadoNormalizado,BGR);
			Mat auxAlpha3[4]={BGR[0],BGR[1],BGR[2],BGRA[3]};
			merge(auxAlpha3,4,resultadoNormalizado);
		}
		imwrite("Imagenes/NDVI/output/"+ strImg +"resultadocolor.png", resultadocolor);
		imwrite("Imagenes/NDVI/output/"+ strImg +"resultadocolornormalizado.png", resultadoNormalizado);
	
		return resultadosalida;
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