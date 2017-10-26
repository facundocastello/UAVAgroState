#ifndef UAVAGROSTATESTICHER_H
#define UAVAGROSTATESTICHER_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "funcionesutiles.h"
#include "CommonFunctions.h"

using namespace cv;
using namespace std;

bool sortByDist(DMatch a, DMatch b){
	return a.distance < b.distance;
}


class UAVAgroStateStitcher{
	public:
		int rowGrid;
		int colGrid;
		float rowOverlap;
		float colOverlap;
		int position;
		int tamano;
		int kPoints;

		UAVAgroStateStitcher(int tamano = 4,
					int position= 1,
					int rowGrid = 5,
					int colGrid = 2,
					float rowOverlap = 0.9,
					float colOverlap = 0.9,
					int kPoints = 300
					)
		{
			this->tamano = tamano;
			this->rowGrid = rowGrid;
			this->colGrid = colGrid;
			this->rowOverlap = rowOverlap;
			this->colOverlap = colOverlap;
			this->position = position;
			this->kPoints = kPoints;
			//position: -abDer = 0 -abIzq = 1 -arDer = 2 -arIzq = 3
		}

		vector<Mat> stitchWarp(Mat scene, Mat obj, Mat homoMatrix){
			Mat  objWarped, imgMaskWarped, imgMask = cv::Mat(obj.size(), CV_8UC1, cv::Scalar(255));;
			warpPerspective(imgMask, imgMaskWarped, homoMatrix, Size(scene.cols, scene.rows));

			warpPerspective(obj, objWarped, homoMatrix, Size(scene.cols, scene.rows));

			Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
			erode(imgMaskWarped, imgMaskWarped, kernel, Point(1, 1), 1);

			objWarped.copyTo(scene, imgMaskWarped);

			return{ scene, imgMaskWarped };
		}

		vector<Mat> stitchWarpTransp(Mat scene, Mat obj, Mat homoMatrix){
			Mat  objWarped, imgMaskWarped, imgMask = cv::Mat(obj.size(), CV_8UC1, cv::Scalar(255));;
			warpPerspective(imgMask, imgMaskWarped, homoMatrix, Size(scene.cols, scene.rows));

			warpPerspective(obj, objWarped, homoMatrix, Size(scene.cols, scene.rows));

			Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
			erode(imgMaskWarped, imgMaskWarped, kernel, Point(1, 1), 1);

			objWarped  = CommonFunctions::makeBackGroundTransparent(objWarped);


			objWarped.copyTo(scene, imgMaskWarped);

			return{ scene, imgMaskWarped };
		}

		// void saveDetectAndCompute(vector<Mat> imgs){
		// 	clock_t begin = clock();
		// 	///CALCULA LOS KEYPOINTS Y DESCRIPTORES DE CADA IMAGEN
		// 	//-- Step 1 and 2 : Detect the keypoints and Calculate descriptors 
		// 	Ptr<cv::BRISK> orb = cv::BRISK::create(3);
		// 	std::vector<KeyPoint> keypoints_1, keypoints_2;
		// 	Mat descriptors_1, descriptors_2;
		// 	cout << "--------------------------------------------------------" << endl;
		// 	cout << "Calculando KeyPoints y descriptores: ... ";
		// 	orb->detectAndCompute(img_1 , mask , keypoints_1 , descriptors_1);
		// 	orb->detectAndCompute(img_2 , mask2 , keypoints_2 , descriptors_2);
		// 	cout << "\n KeyPoint imagen 1: " << keypoints_1.size() << "\n KeyPoint Imagen 2: " << keypoints_2.size() << endl;
		// 	begin = CommonFunctions::tiempo(begin, "Tiempo para kp y descriptores: ");
		// }

		vector<Mat> stitchProcess(Mat img_1, Mat img_2,Mat mask = Mat(), Mat mask2 = Mat())
		{
			Mat H;
			clock_t begin = clock();
			clock_t begintop = clock();
			///CALCULA LOS KEYPOINTS Y DESCRIPTORES DE CADA IMAGEN
			//-- Step 1 and 2 : Detect the keypoints and Calculate descriptors 
			Ptr<cv::BRISK> orb = cv::BRISK::create(3);
			std::vector<KeyPoint> keypoints_1, keypoints_2;
			Mat descriptors_1, descriptors_2;
			cout << "--------------------------------------------------------" << endl;
			cout << "Calculando KeyPoints y descriptores: ... ";
			orb->detectAndCompute(img_1 , mask , keypoints_1 , descriptors_1);
			orb->detectAndCompute(img_2 , mask2 , keypoints_2 , descriptors_2);
			cout << "\n KeyPoint imagen 1: " << keypoints_1.size() << "\n KeyPoint Imagen 2: " << keypoints_2.size() << endl;
			begin = CommonFunctions::tiempo(begin, "Tiempo para kp y descriptores: ");
			///MATCHEA LOS DESCRIPTORES QUE SEAN 'PARECIDOS' ENTRE CADA IMAGEN, USANDO FLANN MATCHER
			//-- Step 3: Matching descriptor vectors using FLANN matcher
			cout << "Realizando Matching: ... \n";
			BFMatcher matcher(NORM_HAMMING,true);
			vector< DMatch > matches;
			matcher.match(descriptors_1, descriptors_2, matches);
			sort(matches.begin(), matches.end(), sortByDist);
			cout << "Matches size: "<<matches.size()<<" \n";
			int matchSize = (matches.size() >=50)?50 : matches.size();
			vector< DMatch > good_matches(matches.begin(),matches.begin() + matchSize);
			///OBTENGO LOS PUNTOS EN LOS QUE SE ENCUENTRAN LOS GOOD MATCHES
			std::vector<Point2f> obj;
			std::vector<Point2f> scene;
			for (int i = 0; i < good_matches.size(); i++) {
				obj.push_back(keypoints_1[good_matches[i].queryIdx].pt);
				scene.push_back(keypoints_2[good_matches[i].trainIdx].pt);
			}
			cout << "good matches: " << good_matches.size() << endl;
			begin = CommonFunctions::tiempo(begin, "Tiempo para match: ");
			// ARMO LA MATRIZ DE HOMOGRAFIA EN BASE A LOS PUNTOS ANTERIORES
			Mat maskH;
			H = findHomography(scene, obj, CV_RANSAC);
			begin = CommonFunctions::tiempo(begin, "Tiempo para RANSAC: ");
			/// DEVUELVO H
			CommonFunctions::tiempo(begintop, "Tiempo total: ");
			cout << "--------------------------------------------------------" << endl;
			return{ H };
		}

		Mat stitchImgs(vector<string> strImgs){
			vector<Mat> H;
			vector<Mat> homoNoMultiplicated;
			clock_t begin = clock();
			CommonFunctions::tiempo(begin, "********************Tiempo en cargar las imagenes: ");
			Mat img0 = CommonFunctions::cargarImagen(strImgs[0], this->tamano);
			//AGREGO INDENTACION

			//CALCULO HOMOGRAFIAS PARA CADA IMAGEN
			H.push_back(Mat::eye(3, 3, CV_64F));
			homoNoMultiplicated.push_back(Mat::eye(3, 3, CV_64F));
			cout << "-|-|-|-|-|-|-|-|-|-|-|-|-|Cargando Homografias: " << endl;
			double xMin=0;double xMax=0;
			double yMin=0;double yMax=0;
			for (int i = 0; i < strImgs.size()-1; i++){
				Mat img1 = CommonFunctions::cargarImagen(strImgs[i+1] , this->tamano);
				cout << "Comenzando a pegar la imagen:" << i<<endl;
				vector<Mat> aux = stitchProcess(img0, img1);
				//Una homografia es para calcular el boundbox (H) y la otra es para
				//con ese boundbox calcular las otras homografias, multiplicandolas 
				//esta es homonomultpilicates
				H.push_back(H[i] * aux[0]);
				H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
				homoNoMultiplicated.push_back(aux[0]);
				//Encuentro el maximo y minimo tanto en x como en y de todas las 
				//homografias para despues poder hacer el bounding box;
				if(H[i+1].at<double>(0,2) < xMin){
					xMin = H[i+1].at<double>(0,2);
					cout<< "xmin" << xMin<<endl;
				}
				if(H[i+1].at<double>(0,2) > xMax){
					xMax = H[i+1].at<double>(0,2);
					cout<< "xmax" << xMax<<endl;
				}
				if(H[i+1].at<double>(1,2) < yMin){
					yMin = H[i+1].at<double>(1,2);
					cout<< "ymin" << yMin<<endl;
				}
				if(H[i+1].at<double>(1,2) > yMax){
					yMax = H[i+1].at<double>(1,2);
					cout<< "ymax" << yMax<<endl;
				}

				CommonFunctions::writeMatOnFile("Data/Homografias/homo" + to_string(i+1) , H[i+1]);
				img0 = img1;
			}
			Mat boundBox = CommonFunctions::cargarImagen(strImgs[0], this->tamano);;
			
			boundBox = CommonFunctions::boundingBox(boundBox, (yMin * -1), yMax , (xMin * -1),xMax);
			
			Mat boundBoxMask;
			threshold( boundBox, boundBoxMask, 1, 255,THRESH_BINARY );
			cvtColor(boundBoxMask, boundBoxMask, cv::COLOR_RGB2GRAY);

			Mat img1 = CommonFunctions::cargarImagen(strImgs[1] , this->tamano);

			vector<Mat> aux = stitchProcess(boundBox, img1,boundBoxMask);
			H[1] = aux[0];
			for(int i = 2 ; i < H.size(); i++){
				H[i]=H[i-1] * homoNoMultiplicated[i];
				// H[i] = H[i] / H[i].at<double>(2,2);
			}
			
			CommonFunctions::tiempo(begin, "********************Tiempo en obtener las homografias: ");
			//USANDO LAS HOMOGRAFIAS, COMIENZO EL PEGADO DE LAS IMAGENES
			cout << "-|-|-|-|-|-|-|-|-|-|-|-|-|Pegando imagenes: ... ("<< strImgs.size()-1<< ")"<< endl;
			for (int i = 1; i < strImgs.size(); i++){
				cout << "imagen: "<< i+1 << endl;
				Mat img = CommonFunctions::cargarImagen(strImgs[i] , this->tamano);
				boundBox = stitchWarp(boundBox, img, H[i])[0];
				string res = "Imagenes/resultados/Pegado/resultados" + to_string(i) + ".png";
				imwrite(res, boundBox);
			}	
			CommonFunctions::tiempo(begin, "********************Tiempo en pegar imagenes: ");

			Mat tmp,alpha;
			
			cvtColor(boundBox,tmp,CV_BGR2GRAY);
			threshold(tmp,alpha,1,255,THRESH_BINARY);
			
			Mat rgb[3];
			split(boundBox,rgb);
			
			Mat rgba[4]={rgb[0],rgb[1],rgb[2],alpha};
			merge(rgba,4,boundBox);

			return boundBox;
		}
		
};

#endif
