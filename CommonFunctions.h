#ifndef COMMONFUNCTIONS_H
#define COMMONFUNCTIONS_H

// #include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video.hpp"
#include <ctime>
#include <dirent.h>

using namespace std;
using namespace cv;

class CommonFunctions{
	public:
		Mat static addIndent(Mat boundBox,
							 int position,
							 int cols,
							 int rows,
							 int colGrid,
							 int rowGrid,
							 float colOverlap,
							 float rowOverlap,
							 int extraIndent = 200
							){
			int izIndent = 
				(position == 0 || position == 2) ?
				rows * colGrid * (1 - colOverlap) : extraIndent;
			int derIndent = 
				(position == 1 || position == 3) ?
				rows * colGrid * (1 - colOverlap) : extraIndent;
			int arIndent = 
				(position == 0 || position == 1) ?
				cols * rowGrid * (1 - rowOverlap) : extraIndent;
			int abIndent = 
				(position == 2 || position == 3) ?
				cols * rowGrid * (1 - rowOverlap) : extraIndent;
			vector<int> indent = {arIndent,abIndent,izIndent,derIndent};
			return  CommonFunctions::boundingBox(boundBox, indent[0], indent[1], indent[2], indent[3]);
		}

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

		vector<int> static setBorder(Mat img){
			Mat dst;
			cvtColor(img, dst, CV_BGR2GRAY);
			CommonFunctions::showWindowNormal(img,"img");
			int min=0;
			int max=10;
			int gaussian=7;
			int dilateSize=3;
			int dilateCant=3;
			namedWindow("gauss", WINDOW_NORMAL);
			createTrackbar( "min", "gauss", &min, 255);
			createTrackbar( "max", "gauss", &max, 255);
			createTrackbar( "gaussian", "gauss", &gaussian, 31);
			createTrackbar( "dilateSize", "gauss", &dilateSize, 31);
			createTrackbar( "dilateCant", "gauss", &dilateCant, 31);
			while(1){
				imshow("gauss", dst);
				int k = waitKey();
				if(k == 27)
        			break;
				cvtColor(img, dst, CV_BGR2GRAY);
				GaussianBlur( dst, dst, Size( gaussian,gaussian ), 0, 0 );
				Canny(dst,dst,min,max,3);
				Mat kernel = getStructuringElement(MORPH_CROSS, Size(dilateSize, dilateSize));
				dilate(dst, dst, kernel, Point(1, 1), dilateCant);
				erode(dst, dst, kernel, Point(1, 1), dilateCant+1);
				min = getTrackbarPos("min","gauss");
				max = getTrackbarPos("max","gauss");
				gaussian = getTrackbarPos("gaussian","gauss");
				dilateSize = getTrackbarPos("dilateSize","gauss");
				dilateCant = getTrackbarPos("dilateCant","gauss");				
			}
			destroyAllWindows();
			vector<int> minMax;
			minMax.push_back(min);
			minMax.push_back(max);
			return minMax;
		}


		Mat static Aindane(Mat inputImage,int sigma){
			Mat I;
			inputImage.convertTo(I,CV_32F);
			// I/=255;

			Mat I_gray;
			cvtColor(inputImage, I_gray, CV_BGR2GRAY);
			vector<Mat> BGRNTSC;
			split(inputImage,BGRNTSC);
			
			Mat I_gray_norm = I_gray.clone();

			I_gray_norm.convertTo(I_gray_norm, CV_32F);
			I_gray_norm = I_gray_norm / 255;
			// 	% Get the threshold that splits the bottom 10% of the CDF
			int histSize = 256;
			float range[] = { 0, 1 } ;
			const float* histRange = { range };
			Mat hist;
			calcHist( &I_gray_norm, 1, 0, Mat(), hist, 1,&histSize, &histRange, true, false );
			// Draw the histograms for B, G and R
			hist.at<float>(0,0) =0;
			Mat cumHist = hist.clone();
			float sumHist=hist.at<float>(0,0);
			for(int i = 1 ;i < cumHist.rows;i++){
				sumHist += hist.at<float>(i,0);
				cumHist.at<float>(i,0) = sumHist;
			}
			Mat cdf = cumHist / sumHist;

			double L;
			for(int i = 1 ;i < cdf.rows;i++){
				if( cdf.at<float>(i,0) >= 0.1 ){
					L = i;
					break;
				}
			}
			// 	% Use the threshold point to compute the z constant
			double z;
			if(L<=50){
				z=0;
			}else if(50 < L || L<=150){
				z=(L-50)/100;
			}else{
				z=1;
			}
			Mat I_nltf = I_gray_norm.clone();
			for(int i=0;i<I_nltf.rows;i++){
				for(int j=0;j<I_nltf.cols;j++){	
					I_nltf.at<float>(i,j)=	(pow(I_nltf.at<float>(i,j),(0.75*z + 0.25))
					+ (1 - I_nltf.at<float>(i,j))*0.4*(1-z)
					+ pow(I_nltf.at<float>(i,j),(2-z))) / 2;
				}
			}
			
			
			int fsize=2*(2*sigma)+1;
			Mat I_gauss;
			GaussianBlur(I_gray_norm,I_gauss,Size(fsize,fsize),sigma,sigma);

			Mat std,mean;
			vector<Mat> BGR;
			split(I,BGR);
			Mat concat;
			hconcat(BGR[0], BGR[1], concat);
			hconcat(concat, BGR[2], concat);
			meanStdDev(concat, mean, std);
			
			cout<<std;
			double p;
			float global_std = std.at<double>(0,0);
			if(global_std <= 3){
				p = 3;
			}else if(3 < global_std && global_std < 10){
				p = (27 - 2*global_std) / 7;
			}else{
				p = 1;
			}
			
			Mat I_ace = I_nltf.clone();
			for(int i=0;i<I_ace.rows;i++){
				for(int j=0;j<I_ace.cols;j++){	
					float iexp = pow((I_gauss.at<float>(i,j) / I_gray_norm.at<float>(i,j)),p);
					I_ace.at<float>(i,j) = 255 * pow( I_ace.at<float>(i,j) , iexp);
				}
			}
			
			for(int i = 0 ; i < 3; i++){
				for(int j = 0 ; j < BGR[i].rows;j++){
					for(int k = 0 ; k < BGR[i].cols;k++){
						BGR[i].at<float>(j,k) =
						(I_ace.at<float>(j,k)-80) * 
						BGR[i].at<float>(j,k) /
						I_gray.at<uchar>(j,k);
					}
				}
			}
			merge(BGR,I);
			I.convertTo(I,CV_8U);
			CommonFunctions::showWindowNormal(I,"res");
			imwrite("Imagenes/Pegado/output/resultadofinalcomp.png",I);

			return inputImage;
		}

		Mat static getBorder(Mat img, vector<int> minMax){
			Mat dst;
			cvtColor(img, dst, CV_BGR2GRAY);
			GaussianBlur( dst, dst, Size( 7,7 ), 0, 0 );
			Canny(dst,dst,minMax[0],minMax[1],3);
			// CommonFunctions::showWindowNormal(dst,"gauss");
			Mat kernel = getStructuringElement(MORPH_CROSS, Size(3, 3));
			dilate(dst, dst, kernel, Point(1, 1), 3);
			erode(dst, dst, kernel, Point(1, 1), 4);
			// CommonFunctions::showWindowNormal(dst,"gauss");
				
			return dst;
			
		}


		vector<Mat> static getBorders(vector<Mat> imgs, vector<int> minMax){
			vector<Mat> borders;
			for(int i = 0 ; i < imgs.size();i++){
				borders.push_back( CommonFunctions::getBorder(imgs[i],minMax) );
			}
			return borders;
		}

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


		bool static crearCarpeta(String str){
			str="mkdir "+ str;
			if( system(str.c_str()) == 0){
				return true;
			}else{
				return false;
			}
		}

		string static obtenerUltimoDirectorio(string &carpeta){
			int indice = carpeta.find_last_of("/");
			string ultimoDir = carpeta.substr(indice+1,carpeta.size());
			carpeta = carpeta.substr(0,indice);
			return ultimoDir;
		}

		void static escribirImagen(string carpeta, Mat img){
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
			imwrite(auxWrite,img);
		}
		//MUESTRA UNA VENTANA CON LA PROPIEDAD DE WINDOW_NORMAL
		void static showWindowNormal(Mat img, String namewindow ="img"){
			namedWindow(namewindow, WINDOW_NORMAL);
			imshow(namewindow, img);
			waitKey();
		}
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

		//OBTIENE EL RECT BOUNDINGBOX DE LA REGION DE INTERES PARA LUEGO CORTARLO
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
		//CORTA UN ROI PASANDOLE EL RECT
		Mat static cropRectorROI(Mat img ,Rect rect){
			Mat auxCrop;
			auxCrop = img(rect);
			img = auxCrop;
			return img;
		}

		//CREA UN BOUNDING BOX INDENTANDO LOS BORDES DE LA IMAGEN QUE SE LE PASA
		Mat static boundingBox(Mat img, int colindent, int rowindent){
			Mat boundingBox(Size(img.cols + colindent*2, img.rows + rowindent*2), CV_8UC3, Scalar(0, 0, 0));
			Mat roibounding(boundingBox, Rect(colindent, rowindent, img.cols, img.rows));
			CommonFunctions::copyToTransparent(img , roibounding);
			return boundingBox;
		}

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

		Mat static makeBackGroundTransparent(Mat img){
			Mat dst;//(src.rows,src.cols,CV_8UC4);
			Mat tmp, alpha;

			cvtColor(img, tmp, CV_BGR2GRAY);
			threshold(tmp, alpha, 1, 255, THRESH_BINARY);

			Mat rgb[3];
			split(img, rgb);

			Mat rgba[4] = { rgb[0], rgb[1], rgb[2], alpha };
			merge(rgba, 4, dst);
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
		  
		

};

#endif