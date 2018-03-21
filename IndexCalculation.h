#ifndef INDEXCALCULATION_H
#define INDEXCALCULATION_H

#include "CommonFunctions.h"


class IndexCalculation{
public:
	double min=0;
	double max=0;

	void processManager(){
		vector<string> strNDVI = CommonFunctions::obtenerImagenes("Imagenes/Indices/input/");
		for(int i = 0 ; i<strNDVI.size();i++){
			indexCalcu(strNDVI[i]);
		}
	}

	void indexCalcu(string strImg){
		Mat imgaux = imread(strImg, IMREAD_UNCHANGED);
		
		size_t position = strImg.find_last_of("/");
		strImg.erase(strImg.begin(),strImg.begin()+position);
		position = strImg.find_last_of(".");
		strImg.erase(strImg.begin()+position,strImg.end());

		

		vector<Mat> BGRA;
		split(imgaux, BGRA);
		//CORRIJO EL PROBLEMA DE QUE EL INFRAROJO 'INVADE' EL ROJO
		// subtract(BGRA[2],BGRA[0]*0.8,BGRA[2],cv::noArray(),CV_8U);
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



		Mat ndviCuantizado = segmentationVariation(ndvi,BGRA[3],5);
		Mat ndviLut = createLut(ndvi,BGRA[3]);
		Mat ndviChart = generarGrafico(ndvi, 50,BGRA[3]);
		CommonFunctions::escribirImagen("Imagenes/Indices/output/"+ strImg +"/ndvi.png", addAlpha(ndvi,BGRA[3]) );
		CommonFunctions::escribirImagen("Imagenes/Indices/output/"+ strImg +"/ndviCuantizado.png", ndviCuantizado );
		CommonFunctions::escribirImagen("Imagenes/Indices/output/"+ strImg +"/ndviLut.png", ndviLut );
		CommonFunctions::escribirImagen("Imagenes/Indices/output/"+ strImg +"/ndviChart.png", ndviChart );
	}
	void rviCalculation(vector<Mat> BGRA, string strImg){
		Mat rvi,numerador,denominador;
		//OBTENGO EL NDVI
		BGRA[0].convertTo(numerador,CV_8S);
		BGRA[2].convertTo(denominador,CV_8S);
		divide(numerador,denominador,rvi,1.,CV_32F);
		//CONVIERTO DE SIGNED A UCHAR
		rvi = (rvi + 1) * 128;
		rvi.convertTo(rvi,CV_8U);

		Mat rviCuantizado = segmentationVariation(rvi,BGRA[3],5);
		Mat rviLut = createLut(rvi,BGRA[3]);
		Mat rviChart = generarGrafico(rvi, 50,BGRA[3]);
		CommonFunctions::escribirImagen("Imagenes/Indices/output/"+ strImg +"/rvi.png", addAlpha(rvi,BGRA[3]) );
		CommonFunctions::escribirImagen("Imagenes/Indices/output/"+ strImg +"/rviCuantizado.png", rviCuantizado );
		CommonFunctions::escribirImagen("Imagenes/Indices/output/"+ strImg +"/rviLut.png", rviLut );
		// CommonFunctions::escribirImagen("Imagenes/Indices/output/"+ strImg +"/rviChart.png", rviChart );
	}

	Mat addAlpha(Mat img, Mat trans){
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

	Mat segmentationVariation(Mat img, Mat trans, int cantColores){
		//recupero fondo blanco
		Mat transAux = trans.clone();
		img = normalizateMat(img,transAux);
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

	void showWindowNormal(Mat img, String namewindow ="img"){
		namedWindow(namewindow, WINDOW_NORMAL);
		imshow(namewindow, img);
		waitKey();
	}

	Mat  createLut(Mat temp, Mat trans){
		temp = normalizateMat(temp, trans);
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
		dst = drawIndexOfIndex(dst,auxDst,trans);

		return dst;
	}

	Mat  drawIndexOfIndex(Mat img,Mat Lut,Mat trans){
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
		int auxMin = min*100/256;
		int auxMax = max*100/256;
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

	Mat normalizateMat(Mat img, Mat mask){
		Mat std,mean,dst;
		// showWindowNormal(img);
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

	Mat generarGrafico(Mat img,int cantidad, Mat trans){
		// showWindowNormal(img);
		int cantPix = 0;
		vector<int> limites;
		//en base a la cantdad de intervalos q quiere genero los limites
		for(int i = 0 ; i <= cantidad; i++){
			limites.push_back(i*256/cantidad);
		}
		vector<int> contador(limites.size()-1,0);
		//cuento la cantidad de pixeles que esten en cada intervalo
		for(int i = 0 ; i < img.rows ; i++){
			for(int j = 0 ; j < img.cols ; j++){
				int pixVal = img.at<uchar>(i,j);
				if(trans.at<uchar>(i,j) == 255){
					cantPix++;
					for(int k = 1;k < limites.size(); k++){
						if(limites[k-1] <= pixVal && pixVal < limites[k]){
							contador[k-1]++;
						}
					}
				}
			}
		}
		//paso todo a porcentaje
		for(int k = 0;k < contador.size(); k++){
			contador[k]/=(cantPix/100);
		}
		//generl el grafico con colores aleatorios
		vector<Vec3b> colors;
		Mat chart(Size(1200,500),CV_32FC3,Scalar(0,0,0));
		int contadorAcum = 0;
		int posText = 5;
		putText(chart, "min  max  porcentaje" , cvPoint(1000,(posText-1)*25), FONT_HERSHEY_PLAIN, 1.1, cvScalar(1,1,1), 1, CV_AA);
		for(int k = 0;k < contador.size();k++){
			if(contador[k] > 0){
				//dibujo pixel a pixel
				Vec3f color;
				color[0] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				color[1] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				color[2] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				Vec3f colorText = color;
				for(int i = chart.rows/2 ; i >= 0 ; i--){
					for(int j = contadorAcum*10; j < (contadorAcum + contador[k])*10-1; j++){
						chart.at<Vec3f>(i,j) = color;
					}
					color[0] *= .995;color[1] *= .995;color[2] *= .995;
				}
				color = colorText;
				for(int i = chart.rows/2 ; i < chart.rows ; i++){
					for(int j = contadorAcum*10; j < (contadorAcum + contador[k])*10-1; j++){
						chart.at<Vec3f>(i,j) = color;
					}
					color[0] *= .995;color[1] *= .995;color[2] *= .995;
				}
				//agrego el texto al grafico

				putText(chart, to_string(limites[k]) , cvPoint(1000,posText*25), FONT_HERSHEY_PLAIN, 1.4, cvScalar(colorText[0],colorText[1],colorText[2]), 1, CV_AA);
				putText(chart, to_string(limites[k+1]) , cvPoint(1050,posText*25), FONT_HERSHEY_PLAIN, 1.4, cvScalar(colorText[0],colorText[1],colorText[2]), 1, CV_AA);
				putText(chart, to_string(contador[k]) + "%" , cvPoint(1130,posText*25), FONT_HERSHEY_PLAIN, 1.6, cvScalar(colorText[0],colorText[1],colorText[2]), 1, CV_AA);
				posText++;
				contadorAcum += contador[k];
			}
		}
		chart*=256;
		chart.convertTo(chart,CV_8U);

		return chart;
	}


    
};


#endif