#ifndef COMMONFUNCTIONS_H
#define COMMONFUNCTIONS_H

#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/gpu/gpu.hpp>
#include <ctime>
#include <dirent.h>
#include "UAVAgroStateIndexCalcs.h"


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
			Mat img = imread(strImg, Tipo);
			resize(img, img, Size(img.cols / tamano, img.rows / tamano));
			if(img.channels() == 3 ){
				img = addTransparence(img);
			}
			return img;
		}

		Mat static getBorder(Mat img){
			Mat dst;
			int scale = 2;
			int delta = 0;
			int ddepth = CV_8U;
			Mat grad;

			int c;
			cvtColor(img, dst, CV_BGR2GRAY);
			GaussianBlur( dst, dst, Size( 3,3 ), 0, 0 );
			/// Generate grad_x and grad_y
			Mat grad_x, grad_y;
			Mat abs_grad_x, abs_grad_y;

			/// Gradient X
			//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
			Sobel( dst, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
			convertScaleAbs( grad_x, abs_grad_x );

			/// Gradient Y
			//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
			Sobel( dst, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
			convertScaleAbs( grad_y, abs_grad_y );


			GaussianBlur( abs_grad_y, abs_grad_y, Size( 5,5 ), 0, 0 );
			threshold(abs_grad_y, abs_grad_y, 20, 255, THRESH_BINARY);
			Mat_<float> kernel1(3,3);
			kernel1 << 0, 0, 0, 1, 1, 1, 0, 0, 0;
			erode(abs_grad_y, abs_grad_y, kernel1, Point(1, 1), 2);	
			dilate(abs_grad_y, abs_grad_y, kernel1, Point(1, 1), 3);
			
			GaussianBlur( abs_grad_x, abs_grad_x, Size( 5,5 ), 0, 0 );
			threshold(abs_grad_x, abs_grad_x, 20, 255, THRESH_BINARY);
			Mat_<float> kernel(3,3);
			kernel << 0, 1, 0, 0, 1, 0, 0, 1, 0;
			erode(abs_grad_x, abs_grad_x, kernel, Point(1, 1), 3);		
			dilate(abs_grad_x, abs_grad_x, kernel, Point(1, 1), 2);

			/// Total Gradient (approximate)
			addWeighted( abs_grad_x, 1, abs_grad_y, 1, 0, grad );
			
			return grad;
			
		}

		vector<Mat> static getBorders(vector<Mat> imgs){
			vector<Mat> borders;
			for(int i = 0 ; i < imgs.size();i++){
				borders.push_back( CommonFunctions::getBorder(imgs[i]) );
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
				imgs.push_back(cargarImagen(strImgs[i], tamano,Tipo));

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
			img.copyTo(roibounding);
			return boundingBox;
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
			img.copyTo(roibounding);
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
			/// Draw for each channel
			for( int i = 1; i < histSize; i++ )
			{
				line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
						Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
						Scalar( 255, 0, 0), 2, 8, 0  );
			}
			/// Display
			namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
			imshow("calcHist Demo", histImage );
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

		  bool static writeMatOnFile(string fileName, Mat content){
			ofstream out(fileName+".txt");
			streambuf *coutbuf = std::cout.rdbuf(); //save old buf
			cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!
			cout << content;  //output to the file out.txt
			cout.rdbuf(coutbuf);
			return true;
		  }

		  Mat static readMatFromFile(string fileName){
			
			std::ifstream in(fileName+".txt");
			std::streambuf *cinbuf = std::cin.rdbuf(); //save old buf
			std::cin.rdbuf(in.rdbuf()); //redirect std::cin to in.txt!
			std::string aux;
			int r = std::count(std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>(), '\n') + 1;
			in.clear();
			in.seekg(0, ios::beg);
			vector<float> content;

			while(cin >> aux)  //input from the file in.txt
			{
				char chars[] = "[],;";
				for (unsigned int i = 0; i < strlen(chars); ++i)
				{
					  // you need include <algorithm> to use general algorithms like std::remove()
					  aux.erase (std::remove(aux.begin(), aux.end(), chars[i]), aux.end());
				}
				content.push_back(stof(aux));
			}
			std::cin.rdbuf(cinbuf);   //reset to standard input again
			int c = content.size() / r;
			Mat M(r,c,CV_32FC1);
			for(int i=0;i<r*c;++i)
			{
				M.at<float>(i)=content[i];
			}

			return M;
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