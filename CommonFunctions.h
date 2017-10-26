#ifndef COMMONFUNCTIONS_H
#define COMMONFUNCTIONS_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/gpu/gpu.hpp>
#include "funcionesutiles.h"
#include <ctime>
#include <dirent.h>


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

		Mat static cargarImagen(string strImg , int tamano = 4){
			Mat img = imread(strImg);
			resize(img, img, Size(img.cols / tamano, img.rows / tamano));
			return img;
		}
		vector<Mat> static cargarImagenes(vector<String> strImgs, int tamano = 4)
		{
			vector<Mat> imgs;
			cout << "-|-|-|-|-|-|-|-|-|-|-|-|-|Cargando Imagenes: (tamaño: "<< strImgs.size()<< ")" << endl;
			for (int i = 0; i < strImgs.size(); i++){
				cout << "Imagen: "<< i+1 << endl;
				imgs.push_back(cargarImagen(strImgs[i], tamano));
			}
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
		clock_t static tiempo(clock_t begin, string msg){
			clock_t end = clock();
			cout << msg << double(end - begin) / CLOCKS_PER_SEC << '\n';
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
			Mat boundingBox(Size(img.cols+ izIndent + derIndent, img.rows + arIndent + abIndent ), CV_8UC3, Scalar(0, 0, 0));
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
		

};

#endif