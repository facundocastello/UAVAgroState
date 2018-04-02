#ifndef COMMONFUNCTIONS_H
#define COMMONFUNCTIONS_H

// #include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include "hpdf.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"
#include <ctime>
#include <dirent.h>

using namespace std;
using namespace cv;
/**
 * @brief Realiza funciones genéricas que pueden ser utiles para cualquier proceso de este software.
 * 
 */
class CommonFunctions{
	public:
		/**
		 * @brief Imprime en pantalla el tamaño, cantidad de canales, y profundidad de una imágen.
		 * 
		 * @param image 
		 * @param out 
		 */
		void static info(const cv::Mat &image, std::ostream &out = std::cout) {
			out << "Characteristics\n";
			out << "\tSize " << image.rows << 'x' << image.cols << '\n';
			out << "\tChannels " << image.channels() << '\n';
			out << "\tDepth ";
			out << '\t';
			switch (image.depth()) {
			case CV_8U: out << "8-bit unsigned integers ( 0..255 )\n"; break;
			case CV_8S: out << "8-bit signed integers ( -128..127 )\n"; break;
			case CV_16U: out << "16-bit unsigned integers ( 0..65535 )\n"; break;
			case CV_16S: out << "16-bit signed integers ( -32768..32767 )\n"; break;
			case CV_32S: out << "32-bit signed integers ( -2147483648..2147483647 )\n"; break;
			case CV_32F: out << "32-bit floating-point numbers ( -FLT_MAX..FLT_MAX, INF, NAN )\n"; break;
			case CV_64F: out << "64-bit floating-point numbers ( -DBL_MAX..DBL_MAX, INF, NAN )\n"; break;
			}
		}
		/**
		 * @brief Carga y redimensiona una imágen en base a su ubicación.
		 * 
		 * @param strImg 
		 * @param tamano 
		 * @param Tipo 
		 * @return Mat 
		 */
		Mat static cargarImagen(string strImg , int tamano = 4, int Tipo = int(IMREAD_UNCHANGED)){
			Mat img;
			try
			{
				img = imread(strImg, Tipo);
				resize(img, img, Size(img.cols / tamano, img.rows / tamano));
				if(img.channels() == 3 ){
					img = addTransparence(img);
				}
			}
			catch( cv::Exception& e )
			{
				cout << endl;
				cout << "\033[1;31m" << "Precaución: " + strImg << " no es una imágen" << "\033[0m" << '\n';
				return Mat();
			}
			return img;
		}

		/**
		 * @brief Carga y redimensiona un grupo de imágenes en base a sus ubicaciones.
		 * 
		 * @param strImgs 
		 * @param tamano 
		 * @param Tipo 
		 * @return vector<Mat> 
		 */
		vector<Mat> static cargarImagenes(vector<string> strImgs, int tamano = 4, int Tipo = int(IMREAD_UNCHANGED))
		{
			vector<Mat> imgs;
			cout << "\033[1;32mCargando Imagenes: (resize: 1/"
			+ to_string(tamano) +" Cantidad:"<< strImgs.size()<<")\033[0m" << endl;
			for (int i = 0; i < strImgs.size(); i++){
				cout << "-" << (i+1) * 100 / strImgs.size() << "%";
				cout.flush();
				Mat img = cargarImagen(strImgs[i], tamano,Tipo);
				if(!img.empty()){
					imgs.push_back(img);
				}
			}
			cout<<endl;
			return imgs;
		}

		/**
		 * @brief Obtiene las ubicaciones de un conjunto de imágenes que estan dentro de una carpeta.
		 * 
		 * @param carpeta 
		 * @param reverse 
		 * @return vector<string> 
		 */
		vector<string> static obtenerImagenes(const char* carpeta , bool reverse = false){
			DIR *dir;
			struct dirent *ent;
			vector<string> argumentos;
			if ((dir = opendir(carpeta)) != NULL) {
				while ((ent = readdir(dir)) != NULL) {
					if(strcmp(".", ent->d_name) && strcmp("..", ent->d_name)) {
						argumentos.push_back(carpeta + string(ent->d_name));
					}
				}
				closedir(dir);
			}
			else {
				/* could not open directory */
				perror("");
			}
			std::sort(argumentos.begin(),argumentos.end());
			if(reverse){
				std::reverse(argumentos.begin(),argumentos.end());	
			}
			for(int i = 0; i < argumentos.size() ; i++){
				cout << (argumentos[i] + '\n');
			}
			return argumentos;
		}

		/**
		 * @brief Crea una carpeta.
		 * 
		 * @param str 
		 * @return true 
		 * @return false 
		 */
		bool static crearCarpeta(String str){
			str="mkdir "+ str;
			if( system(str.c_str()) == 0){
				return true;
			}else{
				return false;
			}
		}
		/**
		 * @brief Obtiene el ultimo directorio (o archivo) de una ubicación y además lo borra de la cadena de texto.
		 * 
		 * @param carpeta 
		 * @return string 
		 */
		string static obtenerUltimoDirectorio(string &carpeta){
			int indice = carpeta.find_last_of("/");
			string ultimoDir = carpeta.substr(indice+1,carpeta.size());
			carpeta = carpeta.substr(0,indice);
			return ultimoDir;
		}
		/**
		 * @brief Obtiene el ultimo directorio (o archivo) de una ubicación sin borrarlo de la cadena de texto.
		 * 
		 * @param carpeta 
		 * @return string 
		 */
		string static obtenerUltimoDirectorio2(string carpeta){
			int indice = carpeta.find_last_of("/");
			string ultimoDir = carpeta.substr(indice+1,carpeta.size());
			carpeta = carpeta.substr(0,indice);
			return ultimoDir;
		}
		/**
		 * @brief Verifica si la ubicación de un archivo existe, en caso contrario crea las carpetas que sean necesarias.
		 * 
		 * @param carpeta 
		 */
		void static manejarCarpeta(string carpeta){
			DIR *dir;
			struct dirent *ent;
			vector<string> crearCarpetas;
			string auxWrite = carpeta;
			obtenerUltimoDirectorio(carpeta);

			while(! ((dir = opendir(carpeta.c_str())) != NULL) ) {
				crearCarpetas.push_back(obtenerUltimoDirectorio(carpeta));
			}
			for(int i =crearCarpetas.size()-1;i >= 0 ; i--){
				carpeta += "/" + crearCarpetas[i];
				crearCarpeta(carpeta);
			}
		}
		/**
		 * @brief Escribe una imágen, creando previamente las carpetas que sean necesarias.
		 * 
		 * @param carpeta 
		 * @param img 
		 */
		void static escribirImagen(string carpeta, Mat img){
			manejarCarpeta(carpeta);
			imwrite(carpeta,img);
		}
		/**
		 * @brief  Muesta una imágen en una ventana.
		 * 
		 * @param img 
		 * @param namewindow 
		 */
		void static showWindowNormal(Mat img, String namewindow ="img"){
			int k = 0;
			while(k!=27){
				namedWindow(namewindow, WINDOW_NORMAL);
				imshow(namewindow, img);
				k = waitKey();
			}
		}
		/**
		 * @brief Devuelve la diferencia en segundos entre un timeval y el timeval actual, e imprime en pantalla esa diferencia.
		 * 
		 * @param begin 
		 * @param msg 
		 * @return timeval 
		 */
		timeval static tiempo(timeval begin, string msg){

			struct timeval end;
			
			double mtime, seconds, useconds;    

			gettimeofday(&end, NULL);
		
			seconds  = end.tv_sec  - begin.tv_sec;
			useconds = end.tv_usec - begin.tv_usec;
		
			mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

			cout << "\033[1;33m" << "Tiempo en " <<msg << mtime/1000 << "\033[0m" << '\n';
			return end;
		}
		/**
		 * @brief Obtiene un rectangulo que indica la caja de límites del area de interes.
		 * 
		 * @param img 
		 * @return Rect 
		 */
		
		Rect static rectROI(Mat img){
			vector<vector<Point> > contours;
			vector<Vec4i> hierarchy;
			Mat auxCrop;
			cvtColor(img, auxCrop, CV_BGR2GRAY);
			findContours(auxCrop, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
			vector<Point> contour = contours[0];
			Rect rectContour = boundingRect(contour);

			return rectContour;
		}
		/**
		 * @brief En base a una imágen y a un rectangulo que indique el area de interes,
		 * corta la imagen.
		 * 
		 * @param img 
		 * @param rect 
		 * @return Mat 
		 */
		Mat static cropRectorROI(Mat img ,Rect rect){
			Mat auxCrop;
			auxCrop = img(rect);
			img = auxCrop;
			return img;
		}
		/**
		 * @brief Agrega a una imágen espacios iguales a la izquierda y derecha, y arriba y abajo.
		 * 
		 * @param img 
		 * @param colindent 
		 * @param rowindent 
		 * @return Mat 
		 */
		Mat static boundingBox(Mat img, int colindent, int rowindent){
			Mat boundingBox(Size(img.cols + colindent*2, img.rows + rowindent*2), CV_8UC3, Scalar(0, 0, 0));
			Mat roibounding(boundingBox, Rect(colindent, rowindent, img.cols, img.rows));
			CommonFunctions::copyToTransparent(img , roibounding);
			return boundingBox;
		}
		/**
		 * @brief Agrega a una imágen espacios arriba, abajo, izquierda y derecha.
		 * 
		 * @param img 
		 * @param arIndent 
		 * @param abIndent 
		 * @param izIndent 
		 * @param derIndent 
		 * @return Mat 
		 */
		Mat static boundingBox(Mat img, int arIndent, int abIndent,int izIndent,int derIndent){
			Scalar scal;
			if(img.channels() == 4){
				scal = Scalar(0, 0, 0, 0);
			}else{
				scal = Scalar(0, 0, 0);
			}
			Mat boundingBox(Size(img.cols+ izIndent + derIndent, img.rows + arIndent + abIndent ), img.type(), Scalar(0, 0, 0));
			Mat roibounding(boundingBox, Rect(izIndent,arIndent, img.cols, img.rows));
			CommonFunctions::copyToTransparent(img , roibounding);
			return boundingBox;
		}
		
		/**
		 * @brief Copia una imágen dentro de otra, pero manteniendo las transparencias.
		 * 
		 * @param obj 
		 * @param scene 
		 * @return Mat 
		 */
		Mat static copyToTransparent(Mat obj, Mat scene){
			for(int i=0;i < obj.rows;i++){
				for(int j=0;j < obj.cols;j++){
					if(obj.at<Vec4b>(i,j)[3] == 255 ){
						scene.at<Vec4b>(i,j)[0] = obj.at<Vec4b>(i,j)[0];
						scene.at<Vec4b>(i,j)[1] = obj.at<Vec4b>(i,j)[1];
						scene.at<Vec4b>(i,j)[2] = obj.at<Vec4b>(i,j)[2];
						scene.at<Vec4b>(i,j)[3] = obj.at<Vec4b>(i,j)[3];					
					}
				}
			}
			return scene;
		}
		/**
		 * @brief Escribe un pdf en una carpeta, creando las carpetas faltantes.
		 * 
		 * @param pdf 
		 * @param str 
		 * @return true 
		 * @return false 
		 */
		bool static escribirPDF(HPDF_Doc pdf, string str){
			
			try{
				manejarCarpeta(str);
				HPDF_SaveToFile (pdf, str.c_str());
			} catch (...) {
				HPDF_Free (pdf);
				return false;
			}

		}
		/**
		 * @brief Agrega transparencia a una imágen, en base a un umbral.
		 * 
		 * @param img 
		 * @return Mat 
		 */
		Mat static addTransparence(Mat img){
			Mat tmp,alpha;
			
			cvtColor(img,tmp,CV_BGR2GRAY);
			threshold(tmp,alpha,1,255,THRESH_BINARY);
			
			Mat rgb[3];
			split(img,rgb);
			
			Mat rgba[4]={rgb[0],rgb[1],rgb[2],alpha};
			merge(rgba,4,img);

			return img;
		}
		/**
		 * @brief Agrega transparencia a una imágen, en base a una mascara.
		 * 
		 * @param img 
		 * @param trans 
		 * @return Mat 
		 */
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


		/**
		 * @brief Quita el cuarto canal de una imágen.
		 * 
		 * @param img 
		 * @return Mat 
		 */
		Mat static removeAlpha(Mat img){
			if(img.channels() == 4){
				vector <Mat> bgra;
				split(img,bgra);
				Mat bgr[3] = {bgra[0],bgra[1],bgra[2]};
				merge(bgr,3,img);
			}
			return img;
		}
		/**
		 * @brief Dibuja el histograma de una imágen.
		 * 
		 * @param img 
		 * @param imgName 
		 */
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
			Mat hist2;
			hist.convertTo(hist2,CV_64F);
		}

		string static type2str(Mat img) {
			int type = img.type();
			string r;
		  
			uchar depth = type & CV_MAT_DEPTH_MASK;
			uchar chans = 1 + (type >> CV_CN_SHIFT);
		  
			switch ( depth ) {
			  case CV_8U:  r = "8U"; break;
			  case CV_8S:  r = "8S"; break;
			  case CV_16U: r = "16U"; break;
			  case CV_16S: r = "16S"; break;
			  case CV_32S: r = "32S"; break;
			  case CV_32F: r = "32F"; break;
			  case CV_64F: r = "64F"; break;
			  default:     r = "User"; break;
			}
		  
			r += "C";
			r += (chans+'0');
		  
			return r;
		  }

		  
		

};

#endif