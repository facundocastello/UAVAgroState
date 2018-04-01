#ifndef Segmentation_H
#define Segmentation_H

#include "CommonFunctions.h"
#include "Color.h"


class Segmentation{
public:
    double min=0;
	double max=0;

	Mat static segmentationVariation(Mat img, Mat trans, int cantColores){
		//recupero fondo blanco
		Mat transAux = trans.clone();
        double min,max;
		img = normalizateMat(img,transAux,min,max);
		int denominador = 256/cantColores;
		transAux.convertTo(transAux,CV_32F);
		img.convertTo(img,CV_32F);
		transAux/=256;
		img/=256;
		img = img.mul(transAux);
		img.convertTo(img,CV_8U,256/denominador,.5);
		img*=denominador;

		return addAlpha(img,trans);
	}

	Mat static  createLut(Mat temp, Mat trans){
        double min,max;
		temp = normalizateMat(temp, trans,min,max);
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
		dst = drawIndexOfIndex(dst,auxDst,trans,min,max);

		return dst;
	}

	Mat static  drawIndexOfIndex(Mat img,Mat Lut,Mat trans,double min, double max){
		//creo imagen blanca
		int sizeIndex = img.cols > img.rows? img.cols : img.rows;
		int resize = 30;
		Mat index(Size(sizeIndex/resize,sizeIndex/(resize/2)),CV_8UC3,Scalar(255,255,255));
		//para agregar el cuado a la transparencia
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
		int auxMin = ((min>0)?min:0)*100/256;
		int auxMax = ((max<256)?max:255)*100/256;
		putText(index, "0."+to_string(auxMin), cvPoint(index.cols*.1,index.rows*.2), 
    	FONT_HERSHEY_TRIPLEX, index.rows/170, cvScalar(0,0,0), 3, CV_AA);
		putText(index, "0."+to_string(auxMin+(auxMax-auxMin)/2), cvPoint(index.cols*.1,index.rows*.55), 
    	FONT_HERSHEY_TRIPLEX, index.rows/200, cvScalar(0,0,0), 3, CV_AA);
		putText(index, "0."+to_string(auxMax), cvPoint(index.cols*.1,index.rows*.9), 
    	FONT_HERSHEY_TRIPLEX, index.rows/170, cvScalar(0,0,0), 3, CV_AA);
		index.copyTo(img(Rect(img.cols-1.1*index.cols,img.rows-1.1*index.rows,index.cols,index.rows)));

		img = addAlpha(img,transAux);
		return img;
	}

	Mat static normalizateMat(Mat img, Mat mask,double &min, double &max){
		Mat std,mean,dst;
		minMaxLoc(img,&min,&max,0,0,mask);
		meanStdDev(img, mean, std, mask);

		// CommonFunctions::histDraw(img,"asd1");
		// dst = img - (mean.at<double>(0,0) - std.at<double>(0,0));
		min = mean.at<double>(0,0) - 4*std.at<double>(0,0);
		max = mean.at<double>(0,0) + 4*std.at<double>(0,0);
		dst = img - min;
		// CommonFunctions::histDraw(dst,"asd2");
		dst = dst * ( 256/(max-min) );
		// CommonFunctions::histDraw(dst,"asd3");
				
		return dst;
	}

	vector<Mat> static generarGrafico(Mat img,int cantidad, Mat trans){
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
		Mat chart(Size(1200,500),CV_32FC3,Scalar(0,0,0));
		float contadorAcum = 0;
		int posText = 2;
		putText(chart, "min  max  porcentaje" , cvPoint(1000,(posText-1)*25), FONT_HERSHEY_PLAIN, 1.1, cvScalar(1,1,1), 1, CV_AA);
		for(int k = 0;k < porcentaje.size();k++){
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

				putText(chart, to_string(limites[k]) , cvPoint(1000,posText*25), FONT_HERSHEY_PLAIN, 1.4, cvScalar(colorText[0],colorText[1],colorText[2]), 1, CV_AA);
				putText(chart, to_string(limites[k+1]) , cvPoint(1050,posText*25), FONT_HERSHEY_PLAIN, 1.4, cvScalar(colorText[0],colorText[1],colorText[2]), 1, CV_AA);
				putText(chart, to_string(porcentaje[k]) + "%" , cvPoint(1130,posText*25), FONT_HERSHEY_PLAIN, 1.6, cvScalar(colorText[0],colorText[1],colorText[2]), 1, CV_AA);
				posText++;
				contadorAcum += porcentaje[k];
			}
		}
		chart*=256;
		chart.convertTo(chart,CV_8U);
		vector<Mat> vecMat;
		vecMat.push_back(chart);
		vecMat.push_back(imgColor);
		return vecMat;
	}

	Mat static addAlpha(Mat img, Mat trans){
		Mat resultado;
		if(!trans.empty() && img.channels() < 4){
			if(img.channels()==1){
				Mat auxAlpha[4]={img*0,img,img*0,trans};
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


	vector<int> static threshMat(Mat img, string str){
			Mat dst = img.clone();
			int min=0;
			int max=10;
			namedWindow(str, WINDOW_NORMAL);
			createTrackbar( "min", str, &min, 255);
			createTrackbar( "max", str, &max, 255);
			while(1){
				imshow(str, dst);
				int k = waitKey();
				if(k == 27)
        			break;
				threshold(img, dst, min, max, THRESH_BINARY);
				min = getTrackbarPos("min",str);
				max = getTrackbarPos("max",str);
			}
			// destroyAllWindows();
			vector<int> minMax;
			minMax.push_back(min);
			minMax.push_back(max);
			return minMax;
	}

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