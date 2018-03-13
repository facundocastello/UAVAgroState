#ifndef UAVAGROSTATEINDEXCALCS_H
#define UAVAGROSTATEINDEXCALCS_H

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
		//CALCULO NDVI
		Mat ndvi = ndviCalculation(BGRA);
		Mat ndviCuantizado = segmentationVariation(ndvi,BGRA[3],5);
		Mat ndviLut = createLut(ndvi,BGRA[3]);
		imwrite("Imagenes/NDVI/output/"+ strImg +"-ndvi.png", addAlpha(ndvi,BGRA[3]) );
		imwrite("Imagenes/NDVI/output/"+ strImg +"-ndviCuantizado.png", addAlpha(ndviCuantizado,BGRA[3]) );
		imwrite("Imagenes/NDVI/output/"+ strImg +"-ndviLut.png", addAlpha(ndviLut,BGRA[3]) );
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
		
		division = (division + 1) * 128;
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
		if(!trans.empty() && img.channels() < 4){
			if(img.channels()==1){
				Mat auxAlpha[4]={img,img,img,trans};
				merge(auxAlpha,4,resultado);
			}else{
				vector<Mat> BGR;
				split(img,BGR);
				Mat auxAlpha[4]={BGR[0],BGR[1],BGR[2],trans};
				merge(auxAlpha,4,resultado);
			}
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

	void static showWindowNormal(Mat img, String namewindow ="img"){
		namedWindow(namewindow, WINDOW_NORMAL);
		imshow(namewindow, img);
		waitKey();
	}

	Mat static createLut(Mat temp, Mat trans){
		temp = normalizateMat(temp, trans);
		Mat M1(1,256,CV_8U);
		Mat M2(1,256,CV_8U);
		Mat M3(1,256,CV_8U);
		for(int i=0;i<256;i++)
		{
			M1.at<uchar>(i)=0;
			if(i<128){
				M2.at<uchar>(i)=2*i-1;
				M3.at<uchar>(i)=255;
			}else{
				M2.at<uchar>(i)=M2.at<uchar>(i-1)-1;
				M3.at<uchar>(i)=M3.at<uchar>(i-1)-2;
			}
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

		std::vector<cv::Mat> planes2;
		planes2.push_back(M1);
		planes2.push_back(M2);
		planes2.push_back(M3);
		Mat auxDst;
		cv::merge(planes2,auxDst);
		// showWindowNormal(auxDst);

		return dst;
	}

	Mat static normalizateMat(Mat img, Mat mask){
		Mat std,mean,dst;
		double min,max;
		minMaxLoc(img,&min,&max,0,0,mask);
		meanStdDev(img, mean, std, mask);
		// dst = img - (mean.at<double>(0,0) - std.at<double>(0,0));
		min = mean.at<double>(0,0) - std.at<double>(0,0) - min;
		dst = img - min;
		
		dst = dst * ( 256/(max-min) );
		
		return dst;
	}

	void static histDraw(Mat img,string imgName){
			/// Establish the number of bins
			int histSize = 256;
			/// Set the ranges ( for B,G,R) )
			float range[] = { 0, 256 } ;
			const float* histRange = { range };
			bool uniform = true; bool accumulate = false;
			Mat hist;
			/// Compute the histograms:
			calcHist( &img, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );
			// Draw the histograms for B, G and R
			int hist_w = 512; int hist_h = 400;
			int bin_w = cvRound( (double) hist_w/histSize );
		
			Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
		
			/// Normalize the result to [ 0, histImage.rows ]
			normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
			/// Draw for each channel
			for( int i = 1; i < histSize; i++ )
			{
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
						Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
						Scalar( 255, 0, 0), 2, 8, 0  );
			}
			/// Display
			showWindowNormal(histImage,imgName);
		}

private:
    
};


#endif