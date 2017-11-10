#ifndef UAVAgroStateStitcher_H
#define UAVAgroStateStitcher_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "funcionesutiles.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
#include "CommonFunctions.h"
using namespace cv;
using namespace std;

bool sortByDist(DMatch a, DMatch b){
	return a.distance < b.distance;
}

class UAVAgroStateStitcher{
	public:
		int tamano;
		bool originalsize=false;
		float kPoints;
		int maxMatch = 100;
		double yMin=0;
		double yMax=0;
		double xMin=0;
		double xMax=0;
		int pondSize;
		int imgHeight;
		int imgWidth;
		int minKeypoints = 5000;
		int tipoHomografia;
		vector<bool> darVuelta;
		vector<Mat> imgs;
		vector<string> strImgs;
		vector<Mat> vecDesc;
		vector< vector<KeyPoint> > vecKp;
		vector< vector< DMatch > > vecMatch[3];
		vector< vector< DMatch > > best_inliers;
		vector<Mat> H;
		vector<Mat> homoNoMultiplicated;
		Mat boundBox;
		double totalError;
		double alfaBA;

		UAVAgroStateStitcher(vector<string> strImgs,
					int tamano = 4,
					float kPoints = 3,
					bool originalsize=false,
					int tipoHomografia =1,
					int pondSize=1
					)
		{
			this->tamano = tamano;
			this->kPoints = kPoints;
			this->originalsize=originalsize;
			this->tipoHomografia = tipoHomografia;
			this->pondSize = pondSize;
			this->strImgs = strImgs;
		}
		/*funcion para pegar una imagen transformada por una homografia
		en otra imagen, en el caso de q tenga 4 canales (o sea el cuarto sea
		alpha [transparente]) hace un pegado especial para que no se pierda
		la transparencia, y en el caso contrario la pega de una manera q no
		se note el paso de una imagen a otra
		*/
		vector<Mat> stitchWarp(Mat scene, Mat obj, Mat homoMatrix){
			Mat  objWarped, imgMaskWarped, imgMask = cv::Mat(obj.size(), CV_8UC1, cv::Scalar(255));;
			warpPerspective(imgMask, imgMaskWarped, homoMatrix, Size(scene.cols, scene.rows));
			warpPerspective(obj, objWarped, homoMatrix, Size(scene.cols, scene.rows));

			Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
			erode(imgMaskWarped, imgMaskWarped, kernel, Point(1, 1), 1);

			if(obj.channels() == 4){
				//en el caso de que haya transparencia, se hace un pegado especial
				Mat objAux(scene.size(), scene.type(),Scalar(0,0,0,0));
				objWarped.copyTo(objAux, imgMaskWarped);
				scene = copyToTransparent(objAux, scene);
			}else{
				Mat objAux(scene.size(), scene.type(),Scalar(0,0,0));
				// objWarped.copyTo(objAux, imgMaskWarped);
				// scene = specialBlending(objAux, scene);
				objWarped.copyTo(scene, imgMaskWarped);
			}

			return{ scene, imgMaskWarped };
		}
		/*
		divide una imagen en todos sus canales, para poder sobre escribir la
		escena, solo en el caso de que el objeto no sea transparente en esa parte
		*/
		Mat copyToTransparent(Mat obj, Mat scene){
			Mat rgbaObj[4];
			split(obj,rgbaObj);
			for(int i=0;i < obj.rows;i++){
				for(int j=0;j < obj.cols;j++){
					if(rgbaObj[3].at<uchar>(i,j) == 255){

						// if(scene.at<Vec4b>(i,j) != Vec4b(0,0,0,0)){
						// 	scene.at<Vec4b>(i,j)[0] = scene.at<Vec4b>(i,j)[0] * 0.5 + obj.at<Vec4b>(i,j)[0] * 0.5;
						// 	scene.at<Vec4b>(i,j)[1] = scene.at<Vec4b>(i,j)[1] * 0.5 + obj.at<Vec4b>(i,j)[1] * 0.5;
						// 	scene.at<Vec4b>(i,j)[2] = scene.at<Vec4b>(i,j)[2] * 0.5 + obj.at<Vec4b>(i,j)[2] * 0.5;
						// 	scene.at<Vec4b>(i,j)[3] = obj.at<Vec4b>(i,j)[3];
						// }else{
							scene.at<Vec4b>(i,j) = obj.at<Vec4b>(i,j);
						// }
					}
				}
			}
			return scene;
		}
		/*
		Es un pegado de un objeto en una escena en el que, en caso de que ambas
		imagenes tengan un	pixel escrito (diferente de negro), se mezclan sus
		valores en la misma proporcion, y en caso de que la escena sea negra y
		el objeto tenga otro valor,	se usa el valor del objeto.
		*/
		Mat specialBlending(Mat obj, Mat scene){
			for(int i=0;i < obj.rows;i++){
				for(int j=0;j < obj.cols;j++){
					if(obj.at<Vec3b>(i,j) != Vec3b(0,0,0)){
						if(scene.at<Vec3b>(i,j) != Vec3b(0,0,0)){
							scene.at<Vec3b>(i,j)[0] = scene.at<Vec3b>(i,j)[0] * 0.5 + obj.at<Vec3b>(i,j)[0] * 0.5;
							scene.at<Vec3b>(i,j)[1] = scene.at<Vec3b>(i,j)[1] * 0.5 + obj.at<Vec3b>(i,j)[1] * 0.5;
							scene.at<Vec3b>(i,j)[2] = scene.at<Vec3b>(i,j)[2] * 0.5 + obj.at<Vec3b>(i,j)[2] * 0.5;
						}else{
							scene.at<Vec3b>(i,j) = obj.at<Vec3b>(i,j);
						}
					}
				}
			}
			return scene;
		}


		double compareMats(Mat scene, Mat obj, Mat homoMatrix){
			Mat objAux,tmp;
			warpPerspective(obj, objAux, homoMatrix, Size(scene.cols, scene.rows));
			threshold(objAux, tmp, 1, 255, THRESH_BINARY);
			double error=0;
			double cantError=0;
			if(scene.channels() == 4){
				for(int i=0;i < objAux.rows;i++){
					for(int j=0;j < objAux.cols;j++){
						if(objAux.at<Vec4b>(i,j) != Vec4b(0,0,0,0) && scene.at<Vec4b>(i,j) != Vec4b(0,0,0,0)){
							error += abs( scene.at<Vec4b>(i,j)[0] - objAux.at<Vec4b>(i,j)[0] );
							error += abs( scene.at<Vec4b>(i,j)[1] - objAux.at<Vec4b>(i,j)[1] );
							error += abs( scene.at<Vec4b>(i,j)[2] - objAux.at<Vec4b>(i,j)[2] );
							cantError++;
						}
					}
				}
			}else{
				for(int i=0;i < objAux.rows;i++){
					for(int j=0;j < objAux.cols;j++){
						if(objAux.at<Vec3b>(i,j) != Vec3b(0,0,0) && scene.at<Vec3b>(i,j) != Vec3b(0,0,0)){
							error += abs( scene.at<Vec3b>(i,j)[0] - objAux.at<Vec3b>(i,j)[0] );
							error += abs( scene.at<Vec3b>(i,j)[1] - objAux.at<Vec3b>(i,j)[1] );
							error += abs( scene.at<Vec3b>(i,j)[2] - objAux.at<Vec3b>(i,j)[2] );
							cantError++;
						}
					}
				}
			}
			// objAux = abs(scene - objAux);
			// CommonFunctions::showWindowNormal(objAux);
			// cout<< "eerror: " << error / cantError << endl;

			return (error / cantError);
		}
		/*
		usando paralelismo obtengo todos los keypoints y descriptores
		*/
		void detectAndDescript(){
			cout << "\033[1;32mObteniendo keypoints y descriptores: \033[0m"<< endl;
			///CALCULA LOS KEYPOINTS Y DESCRIPTORES DE CADA IMAGEN
			//-- Step 1 and 2 : Detect the keypoints and Calculate descriptors
			vecDesc = vector<Mat>(imgs.size());
			vecKp = vector< vector<KeyPoint> >(imgs.size());
			parallel_for_(Range(0, imgs.size()), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					float kTres = this->kPoints;
					std::vector<KeyPoint> keypoints;
					Mat descriptors;
					while(keypoints.size() < minKeypoints && kTres >= 0){
						Ptr<cv::BRISK> orb = cv::BRISK::create(kTres);
						// Ptr<cv::AKAZE> orb = cv::AKAZE::create(
							// AKAZE::DESCRIPTOR_MLDB,0,3, .000001);
						Mat mask = Mat();
						Mat tmp;

						if(imgs[i].channels() == 4){
							cvtColor(imgs[i], tmp, CV_BGRA2GRAY);
							threshold(tmp, mask, 1, 255, THRESH_BINARY);
							Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
							erode(mask, mask, kernel, Point(1, 1), 20);
						}
						orb->detectAndCompute(imgs[i] , mask , keypoints , descriptors);
						kTres--;
					}
					cout << "\n KeyPoint imagen" + to_string(i) + ": " << keypoints.size();
					vecDesc[i]=descriptors;
					vecKp[i]=keypoints;
				}
			});
			cout << endl;
		}
		/*Realiza matchs entre los keypoints de 2 imagenes, en base a sus
		descriptores, y esto es acelerado usando paralelismo*/
		void matchKp(){
			cout << "\033[1;32m Matcheando: \033[0m" << endl;
			vecMatch[0] = vector< vector< DMatch > >(imgs.size()-1);
			vecMatch[1] = vector< vector< DMatch > >(imgs.size()-1);
			vecMatch[2] = vector< vector< DMatch > >(imgs.size()-1);
			best_inliers = vector< vector< DMatch > >(imgs.size()-1);
			darVuelta = vector<bool> (imgs.size() -1 );
			BFMatcher matcher(NORM_HAMMING,true);
			parallel_for_(Range(0, vecDesc.size()-1), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					// cout << "Empezo Match "+to_string(i)+ ". \n";
					vector< DMatch > matches;
					matcher.match(vecDesc[i],vecDesc[i+1], vecMatch[0][i]);
					// if( i < (vecDesc.size()-2)  ){
					// 	matcher.match(vecDesc[i],vecDesc[i+2], vecMatch[1][i]);
					// }
					// if( i < (vecDesc.size()-3) ){
					// 	matcher.match(vecDesc[i],vecDesc[i+3], vecMatch[2][i]);
					// }
					cout << "Termino Match "+to_string(i)+ " con "+ to_string(vecMatch[0][i].size()) +".\n" ;
				}
			});
		}
		/*
		elimino matches 'erroneos' usando como criterio para saber si son malos
		o buenos el hecho de que entre cada imagen hay un desplazamiento
		*/
		vector< DMatch > goodMatches(int numMatch,double porcMinY,double porcMaxY,double porcMinX,double porcMaxX){
			vector< DMatch > gm;
			bool imgMismaAltura = false;
			int minvaly = imgHeight*porcMinY;
			int maxvaly = imgHeight*porcMaxY;
			int minvalx = imgWidth*porcMinX;
			int maxvalx = imgWidth*porcMaxX;
			// int minvalx = imgWidth*porcMinX;
			// int maxvalx = imgWidth*porcMaxX;
			while(gm.size() < 4){
				gm= vector< DMatch >();
				for(int i = 0; i < vecMatch[0][numMatch].size(); i++){
					Point2f pt0 = vecKp[numMatch][vecMatch[0][numMatch][i].queryIdx].pt;
					Point2f pt1 = vecKp[numMatch+1][vecMatch[0][numMatch][i].trainIdx].pt;
					if( (pt1.y-pt0.y) > minvaly && (pt1.y-pt0.y) < maxvaly
						&& (pt1.x-pt0.x) >= minvalx && (pt1.x-pt0.x) < maxvalx
						){
						gm.push_back(vecMatch[0][numMatch][i]);
					}
				}
				if(imgMismaAltura){
					break;
				}
				if(maxvaly>5000){
					minvaly = 0;
					maxvaly = imgHeight*porcMinY;
					imgMismaAltura = true;
				}
				minvaly += imgHeight*porcMinY;
				maxvaly += imgHeight*porcMaxY;
			}
			if(gm.size() < 200){
				sort(vecMatch[0][numMatch].begin(),vecMatch[0][numMatch].end(),sortByDist);
				gm.insert(gm.end(),vecMatch[0][numMatch].begin(),vecMatch[0][numMatch].begin()+50);
			}
			// cout<<maxvalx<<endl;
			return gm;
		}

		Mat rigidToHomography(Mat R){

			Mat Homography = Mat();
			if(R.cols != 0){
				Homography = cv::Mat(3,3,R.type());
				Homography.at<double>(0,0) = R.at<double>(0,0);
				Homography.at<double>(0,1) = R.at<double>(0,1);
				Homography.at<double>(0,2) = R.at<double>(0,2);

				Homography.at<double>(1,0) = R.at<double>(1,0);
				Homography.at<double>(1,1) = R.at<double>(1,1);
				Homography.at<double>(1,2) = R.at<double>(1,2);

				Homography.at<double>(2,0) = 0.0;
				Homography.at<double>(2,1) = 0.0;
				Homography.at<double>(2,2) = 1.0;
			}

			return Homography;
		}

		vector< DMatch > removeOutliers(vector< DMatch > gm,  int numHomo, int numHomo2){
			double mediaX=0;
			double mediaY=0;
			vector< DMatch > aux = gm;
			// for(int i = 0 ; i < vecMatch[0][numHomo].size();i++){
			for(int i=0; i < gm.size(); i++){
				Point2f pt0 = vecKp[numHomo][gm[i].queryIdx].pt;
				Point2f pt1 = vecKp[numHomo2][gm[i].trainIdx].pt;
				mediaX += pt1.x - pt0.x;
				mediaY += pt1.y - pt0.y;
			}
			mediaX /= gm.size();
			mediaY /= gm.size();
			// cout << "Mediax " << mediaX << " MediaY "<< mediaY;
			// }
			double varianzaX=0;
			double varianzaY=0;
			for(int i = 0 ; i < gm.size();i++){
				Point2f pt0 = vecKp[numHomo][gm[i].queryIdx].pt;
				Point2f pt1 = vecKp[numHomo2][gm[i].trainIdx].pt;
				varianzaX += abs(mediaX - (pt1.x - pt0.x));
				varianzaY += abs(mediaY - (pt1.y - pt0.y));
			}
			varianzaX /= gm.size();
			varianzaY /= gm.size();
			// cout << " varianzaX " << varianzaX << " varianzaY "<< varianzaY;
			for(int i = 0 ; i < gm.size();i++){
				Point2f pt0 = vecKp[numHomo][gm[i].queryIdx].pt;
				Point2f pt1 = vecKp[numHomo2][gm[i].trainIdx].pt;
				double x = (pt1.x - pt0.x);
				double y = (pt1.y - pt0.y);
				if( !(abs(x - mediaX) < varianzaX && abs(y - mediaY) < varianzaY) ){
					gm.erase(gm.begin() + i);
					i--;
				}
			}
			if(gm.size() < 3){
				return aux;
			}
			return gm;
		}

		/*
		obtengo varias homografias modificando ciertos parametros, y elijo la que
		sea mas adecuada
		*/
		void getHomography(int numHomo, bool usarError)
		{
			double minHomoX = 9999;
			double minError = 9999;
			double minHomoY=9999;
			double porcMinY = 0.05;
			double porcMaxY = 0.5;
			double minvalx;
			double maxvalx;
			int bestvalx;double bestPorcMinY;double bestPorcMaxY;
			int vueltasI=400;int vueltasJ=10;int vueltasK=10;
			vector< DMatch > best_matches;
			vector<double> auxError(vueltasI);
			vector<double> auxHomoX(vueltasI);
			vector<double> auxHomoY(vueltasI);
			Mat MaskInliers;


			sort(vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].end(),sortByDist);

			parallel_for_(Range(0, vueltasI), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					porcMinY = 0.00;
					vector< DMatch > goodm;
					goodm.insert(goodm.end(),vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].begin()+3+i);
					goodm = removeOutliers(goodm, numHomo,numHomo+1);
										
					std::vector<Point2f> obj;
					std::vector<Point2f> scene;
					for (int l = 0; l < goodm.size(); l++) {
						obj.push_back(vecKp[numHomo][goodm[l].queryIdx].pt);
						scene.push_back(vecKp[numHomo+1][goodm[l].trainIdx].pt);
					}
					
					Mat auxH = rigidToHomography( estimateRigidTransform(scene,obj,false) );
					if(!auxH.empty()){
						Mat prodH=H[numHomo]*auxH;
					// 	// por trigonometria aplico lo siguiente
						auxHomoX[i] = abs( pow(prodH.at<double>(0,0),2) + pow(prodH.at<double>(0,1),2) -1);
						auxHomoY[i] = abs( pow(prodH.at<double>(1,0),2) + pow(prodH.at<double>(1,1),2) -1);
					}else{
						auxHomoY[i] = 9999;	
						auxHomoX[i] = 9999;	
						auxError[i] = 9999;
					}
				}
			});
			for(int i=0;i<vueltasI;i++){
					if( /*auxError[i] <= minError &&*/ auxHomoX[i] < minHomoX && auxHomoY[i] < minHomoY){
						bestvalx = i;
						minError = auxError[i];
						minHomoX = auxHomoX[i];
						minHomoY = auxHomoY[i];
					}
			}
			totalError += minError;
			best_matches.insert(best_matches.end(),vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].begin()+3+bestvalx);
			best_matches = removeOutliers(best_matches, numHomo,numHomo+1);
			std::vector<Point2f> obj;
			std::vector<Point2f> scene;
			for (int l = 0; l < best_matches.size(); l++) {
				obj.push_back(vecKp[numHomo][best_matches[l].queryIdx].pt);
				scene.push_back(vecKp[numHomo+1][best_matches[l].trainIdx].pt);
			}
			homoNoMultiplicated[numHomo+1] = rigidToHomography( estimateRigidTransform(scene,obj,false) );
		
			if(minHomoX == 9999){
				cout << "Homografia vacia: " << numHomo+1 << endl;
				homoNoMultiplicated[numHomo+1] = (Mat::eye(3, 3, CV_64F));
			}
			Mat aux;
			// cout << " " << bestvalx << endl;
			Mat img1 = imgs[numHomo];
			Mat img2 = imgs[numHomo+1];
			// if(img1.channels() == 4){
			// 	img1 = removeAlpha(imgs[numHomo]);
			// 	img2 = removeAlpha(imgs[numHomo+1]);
			// }
			drawMatches(img1, vecKp[numHomo],img2,vecKp[numHomo+1],best_matches,aux,
			Scalar::all(-1),Scalar::all(-1),
			std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(numHomo)+".png",aux);
			best_inliers[numHomo] =  best_matches;
		}
		/*
		obtengo varias homografias modificando ciertos parametros, y elijo la que
		sea mas adecuada
		*/

		Mat removeAlpha(Mat img){
			if(img.channels() == 4){
				vector <Mat> bgra;
				split(img,bgra);
				Mat bgr[3] = {bgra[0],bgra[1],bgra[2]};
				merge(bgr,3,img);
			}
			return img;
		}

		void getHomographyOrthomosaic(int numHomo, bool vertical)
		{
			double minHomoX = 9999;
			double minError = 9999;
			double minHomoY=9999;
			double porcMinY = 0.05;
			double porcMaxY = 0.5;
			double minvalx;
			double maxvalx;
			int bestvalx;double bestPorcMinY;double bestPorcMaxY;
			int vueltasI=1000;int vueltasJ=10;int vueltasK=10;
			vector< DMatch > best_matches;
			vector<double> auxError(vueltasI);
			vector<double> auxHomoX(vueltasI);
			vector<double> auxHomoY(vueltasI);
			Mat MaskInliers;


			sort(vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].end(),sortByDist);

			parallel_for_(Range(0, vueltasI), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					porcMinY = 0.00;
					vector< DMatch > goodm;
					goodm.insert(goodm.end(),vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].begin()+4+i);
					goodm = removeOutliers(goodm, numHomo,numHomo+1);
										
					std::vector<Point2f> obj;
					std::vector<Point2f> scene;
					for (int l = 0; l < goodm.size(); l++) {
						obj.push_back(vecKp[numHomo][goodm[l].queryIdx].pt);
						scene.push_back(vecKp[numHomo+1][goodm[l].trainIdx].pt);
					}
					
					Mat auxH = findHomography(scene,obj,CV_RANSAC);
					if(!auxH.empty()){
						Mat prodH=H[numHomo]*auxH;
					// 	// por trigonometria aplico lo siguiente
						// auxError[i] = getActualHomographyError(numHomo,auxH,goodm);
						auxHomoX[i] = abs( pow(prodH.at<double>(0,0),2) + pow(prodH.at<double>(0,1),2) -1);
						auxHomoY[i] = abs( pow(prodH.at<double>(1,0),2) + pow(prodH.at<double>(1,1),2) -1);
					}else{
						auxHomoY[i] = 9999;	
						auxHomoX[i] = 9999;	
						auxError[i] = 9999;
					}
				}
			});
			for(int i=0;i<vueltasI;i++){
					if( auxError[i] <= minError /*&& auxHomoX[i] < minHomoX && auxHomoY[i] < minHomoY */){
						bestvalx = i;
						minError = auxError[i];
						minHomoX = auxHomoX[i];
						minHomoY = auxHomoY[i];
					}
			}
			best_matches.insert(best_matches.end(),vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].begin()+3+bestvalx);
			best_matches = removeOutliers(best_matches, numHomo, numHomo+1);
			
			std::vector<Point2f> obj;
			std::vector<Point2f> scene;
			for (int l = 0; l < best_matches.size(); l++) {
				obj.push_back(vecKp[numHomo][best_matches[l].queryIdx].pt);
				scene.push_back(vecKp[numHomo+1][best_matches[l].trainIdx].pt);
			}
			homoNoMultiplicated[numHomo+1] = findHomography(scene,obj,CV_RANSAC);
		
			if(minHomoX == 9999){
				cout << "Homografia vacia: " << numHomo+1 << endl;
				homoNoMultiplicated[numHomo+1] = (Mat::eye(3, 3, CV_64F));
			}
			cout << " " << bestvalx << endl;
			Mat aux;
			best_inliers[numHomo] = best_matches;
			drawMatches(removeAlpha(imgs[numHomo]), vecKp[numHomo],removeAlpha(imgs[numHomo+1]),vecKp[numHomo+1],best_matches,aux,
			Scalar::all(-1),Scalar::all(-1),
			std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(numHomo)+".png",aux);
		}

		void getHomographies(){
			cout << "\033[1;32mGenerando homografias: \033[0m" << endl;
			H = vector<Mat>(imgs.size());
			homoNoMultiplicated = vector<Mat>(imgs.size());
			H[0] = (Mat::eye(3, 3, CV_64F));
			homoNoMultiplicated[0] = (Mat::eye(3, 3, CV_64F));
			alfaBA = 1000;
			for(int i = 0; i < imgs.size()-1;i++){
				cout << "Realizando Homografia "+to_string(i)+ ". \n";
				switch(tipoHomografia){
					case 1:{
						//caso comun
						getHomography(i,false);
					}
					break;
					case 2:{
						//pego 2 ortomosaicos con solapamiento vertical
						getHomographyOrthomosaic(i,1);
					}
					break;
					case 3:{
						//pego 2 ortomosaicos con solapamiento horizontal
						getHomographyOrthomosaic(i,0);
					}
					break;
				}
				H[i+1] = (H[i] * homoNoMultiplicated[i+1]);
				H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
			}
			alfaBA = 2;
			for(int j = 0; j < 1 ;j++){
				totalError = 0;
				for(int i = 0; i < imgs.size()-1;i++){
					// cout << "Realizando Homografia "+to_string(i)+ ". \n";
					//caso comun
					homoNoMultiplicated[i+1] = getActualHomographyError(i,homoNoMultiplicated[i+1],best_inliers[i]);
					// getHomography(i,true);
					H[i+1] = (H[i] * homoNoMultiplicated[i+1]);
					H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
				}
				cout<< "error total: " << totalError << endl;
			}
		}
		/*
		En base a las homografias, se obtienen los valores que van a delimitar al bound box,
		y evalua que estos no sean de una homogragia mal calculada
		*/
		bool findBoundBoxLimits(){
			// cout << "\033[1;32mObteniendo bordes boundbox: \033[0m"<< endl;
			// FileStorage fsHomo("Data/Homografias/homografias.yml", FileStorage::WRITE);
			// for(int i = 0; i < H.size()-1; i++){
			// 	double newMinY = H[i+1].at<double>(1,2) +( (H[i+1].at<double>(1,0) < 0)? imgWidth * H[i+1].at<double>(1,0) : 0 );
			// 	double newMinX = H[i+1].at<double>(0,2) +( (H[i+1].at<double>(0,1) < 0)? imgHeight * H[i+1].at<double>(0,1) : 0 );
			// 	double newMaxY = imgWidth * H[i+1].at<double>(1,0) + imgHeight * H[i+1].at<double>(1,1) + H[i+1].at<double>(1,2) - imgHeight;
			// 	double newMaxX = imgWidth * H[i+1].at<double>(0,0) + imgHeight * H[i+1].at<double>(0,1) + H[i+1].at<double>(0,2) - imgWidth;
			// 	if(newMinX < xMin){
			// 		xMin = newMinX;
			// 	}
			// 	if(newMaxX > xMax){
			// 		xMax = newMaxX;
			// 	}
			// 	if(newMinY < yMin){
			// 		yMin = newMinY;
			// 	}
			// 	if(newMaxY > yMax){
			// 		yMax = newMaxY;
			// 	}
			// 	fsHomo << "homografia"+to_string(i+1) << H[i+1];
			// }
			// fsHomo.release();
			yMax += 1000;
			xMax += 5000;
			cout<< "ymin: "<< yMin << " ymax: "<< yMax<< "xmin: "<< xMin << " xmax: "<< xMax << endl;
		}

		bool evaluateHomography(){
			if(abs(yMin) > (imgHeight * imgs.size()/2)	|| 	abs(yMax) > (imgHeight * imgs.size()/2)
			|| 	abs(xMin) > (imgWidth * imgs.size()/2)	|| 	abs(xMax) > (imgWidth * imgs.size()/2)){
				cout<< " mal pegado "<<(abs(yMin) > (imgHeight * imgs.size()/2) )
				<<(abs(yMax) > (imgHeight* imgs.size()/2))<<(abs(xMin) > (imgWidth * imgs.size()/2))
				<<(abs(xMax) > (imgWidth * imgs.size()/2))<<endl;
				return true;
			}
			return true;
		}
		/*
		usando los boundboxlimits obtenidos en la funcion "findBoundBoxLimits", se genera
		el boundbox
		*/
		void generateBoundBox(){
			cout << "\033[1;32m Generando boundbox y calculando su homografia: \033[0m" << endl;
			/* Le agrego a la imagen inicial los bordes con el suficiente espacio para
			poder pegar todas las imagenes */
			boundBox = imgs[0];
			boundBox = CommonFunctions::boundingBox(boundBox, abs(yMin) , yMax , abs(xMin),xMax);
			//adapto los keypoints de la primer imagen, al boundbox generado con esta
			Point2f ptAux(abs(xMin),abs(yMin));
			for(int i=0;i<vecKp[0].size();i++){
				vecKp[0][i].pt+=ptAux;
			}
			//vuelvo a calcular la homografia para la imagen con bordes
			imgs[0] = boundBox;
			std::vector<Point2f> obj;
			std::vector<Point2f> scene;
			for (int l = 0; l < best_inliers[0].size(); l++) {
				obj.push_back(vecKp[0][best_inliers[0][l].queryIdx].pt);
				scene.push_back(vecKp[1][best_inliers[0][l].trainIdx].pt);
			}

			if(tipoHomografia == 1){
				homoNoMultiplicated[1] = rigidToHomography( estimateRigidTransform(scene,obj,false) );
			}else{
				homoNoMultiplicated[1] = findHomography(scene,obj,CV_RANSAC);
			}

			H[1] = homoNoMultiplicated[1];

			/*tengo que adaptar todas las homografias a las nuevas dimensiones
			definidas por el boundbox*/
			for(int i = 2 ; i < H.size(); i++){
				H[i] = H[i-1] * homoNoMultiplicated[i];
				H[i] = H[i] / H[i].at<double>(2,2);
			}
		}
		/*
		En el caso de que las homografias se hayan calculado en base a imagenes a las cuales
		se les cambio el tamaño para que sea mas rapido el procesamiento, se les modifica la
		homografia para adaptarlas a su tamaño
		*/
		void rescaleHomographies(){
			if(originalsize){
				cout << "\033[1;32m Resize al tamaño original: \033[0m" << endl;
				FileStorage fsHomo("Data/Homografias/homografias2.yml", FileStorage::WRITE);

				Mat project_down = (Mat::eye(3, 3, CV_64F));
				project_down.at<double>(0,0)/=tamano;
				project_down.at<double>(1,1)/=tamano;
				project_down.at<double>(0,2)/=(tamano*(tamano/2));
				project_down.at<double>(1,2)/=(tamano*(tamano/2));
				Mat project_up = (Mat::eye(3, 3, CV_64F));
				project_up.at<double>(0,0)*=tamano;
				project_up.at<double>(1,1)*=tamano;
				project_up.at<double>(0,2)/=(tamano);
				project_up.at<double>(1,2)/=(tamano);
				// fsHomo << "scaleHomo" << scaleHomo;
				for(int i = 0 ; i < H.size(); i++){
					H[i]= project_up * H[i] * project_down;
					fsHomo << "homografia"+to_string(i) << H[i];
				}
				yMin*=tamano;yMax*=tamano;xMin*=tamano;xMax*=tamano;
				imgs = CommonFunctions::cargarImagenes(strImgs , 1);
				fsHomo.release();

				boundBox = imgs[0];
				boundBox = CommonFunctions::boundingBox(boundBox, abs(yMin) , yMax , abs(xMin),xMax);
			}
		}

		struct SnavelyReprojectionError {
			SnavelyReprojectionError(double observed_x, double observed_y,double to_project_x,double to_project_y)
				: observed_x(observed_x), observed_y(observed_y),to_project_x(to_project_x),to_project_y(to_project_y) {}

			template <typename T>
			bool operator()(const T* const camera,
							T* residuals) const {

				// camera[0,1,2] are the angle-axis rotation.
				T p[2];
				// ceres::AngleAxisRotatePoint(camera, point, p);
				p[0]=camera[0]*T(to_project_x) + camera[1]*T(to_project_y) + camera[2];
				p[1]=camera[3]*T(to_project_x) + camera[4]*T(to_project_y) + camera[5];

				// The error is the difference between the predicted and observed position.
				residuals[0] = p[0] - T(observed_x);
				residuals[1] = p[1] - T(observed_y);
				return true;
			}

			// Factory to hide the construction of the CostFunction object from
			// the client code.
			static ceres::CostFunction* Create(const double observed_x,
												const double observed_y,
												const double to_project_x,
												const double to_project_y) {
				return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2,6>(
							new SnavelyReprojectionError(observed_x, observed_y,to_project_x,to_project_y)));
			}

			double observed_x;
			double observed_y;
			double to_project_x;
			double to_project_y;
			};

		double projectPointError(Point2f pt1 , Point2f pt2, Mat hAux){
			Mat X1 = (Mat_<double>(3,1) << pt1.x, pt1.y, 1);
			Mat X2 = (Mat_<double>(3,1) << pt2.x, pt2.y, 1);
			double x = hAux.at<double>(0,0)*X2.at<double>(0,0)+
			hAux.at<double>(0,1)*X2.at<double>(0,1)+
			hAux.at<double>(0,2)*X2.at<double>(0,2);
			double y = hAux.at<double>(1,0)*X2.at<double>(0,0)+
			hAux.at<double>(1,1)*X2.at<double>(0,1)+
			hAux.at<double>(1,2)*X2.at<double>(0,2);

			double rX = pow(x - X1.at<double>(0,0),2);
			double rY = pow(y - X1.at<double>(0,1),2);
			double valAbs = sqrt(rX+rY);
			double cuadValAbs = pow(valAbs,2);
			double r;
			if(valAbs < alfaBA){
				r= cuadValAbs;
			}else{
				r = 2*alfaBA*valAbs - alfaBA*alfaBA;
			}
			return r;
		}


		Mat getActualHomographyError(int i, Mat homo, vector< DMatch > gm){
			double errorInlier = 0;
			int numMatches=0;

			Mat hAux = homo.clone(); // eye matrix
			// j == 0
			double hH[6];
			hH[0]=hAux.at<double>(0,0);
			hH[1]=hAux.at<double>(0,1);
			hH[2]=hAux.at<double>(0,2);
			hH[3]=hAux.at<double>(1,0);
			hH[4]=hAux.at<double>(1,1);
			hH[5]=hAux.at<double>(1,2);
			Problem problem;
			for(int k = 0 ; k < gm.size();k++){
				Point2f pt1 = vecKp[i][gm[k].queryIdx].pt;
				Point2f pt2 = vecKp[i+1][gm[k].trainIdx].pt;
				ceres::CostFunction* cost_function =
				SnavelyReprojectionError::Create(
					pt1.x,
					pt1.y,
					pt2.x,
					pt2.y);

				problem.AddResidualBlock(cost_function,
									 NULL /* squared loss */,
									 hH);
				errorInlier += 	projectPointError(pt1,pt2,hAux);
			}


			ceres::Solver::Options options;
			options.linear_solver_type = ceres::DENSE_SCHUR;
			options.minimizer_progress_to_stdout = true;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			std::cout << summary.FullReport() << "\n";

			hAux.at<double>(0,0)=hH[0];
			hAux.at<double>(0,1)=hH[1];
			hAux.at<double>(0,2)=hH[2];
			hAux.at<double>(1,0)=hH[3];
			hAux.at<double>(1,1)=hH[4];
			hAux.at<double>(1,2)=hH[5];


			numMatches+= gm.size();
			// cout << errorInlier;
			return hAux;
		}
		
		double getAfterHomographyError(int i, Mat homo){
			double errorInlier = 0;
			int maxMatches = 2;
			int afterMatches = ((imgs.size()-1 - i) > (maxMatches+1))? (maxMatches+1) : imgs.size()-1 - i ;
			int numMatches=0;

			Mat hAux = homo.clone(); // eye matrix

			for(int j = 1; j < afterMatches; j++){
				hAux *= homoNoMultiplicated[i+1+j];
				vector< DMatch > auxMatches;
				vector< DMatch > auxBestMatches;
				auxBestMatches = vecMatch[j][i];
				sort(auxBestMatches.begin(),auxBestMatches.end(),sortByDist);
				auxBestMatches.erase(auxBestMatches.begin()+50,auxBestMatches.end());
				auxBestMatches = removeOutliers(auxBestMatches,i,i+1+j);
				for(int k = 0 ; k < auxBestMatches.size();k++){
					Point2f pt1 = vecKp[i][auxBestMatches[k].queryIdx].pt;
					Point2f pt2 = vecKp[i+1+j][auxBestMatches[k].trainIdx].pt;
					double error = 	projectPointError(pt1,pt2,hAux);
					errorInlier += 	error;
				}
					numMatches+=auxBestMatches.size();
			}
			if(numMatches == 0){
				numMatches ++;
			}

			return (errorInlier/numMatches);
		}

		double getBeforeHomographyError(int i, Mat homo){
			double errorInlier = 0;
			int maxMatches = 2;
			int beforeMatches = (i < maxMatches)? i : maxMatches;
			int numMatches=0;

			Mat hAux = homo.clone();
			for(int j = -1; j > -(beforeMatches+1) ; j--){
				vector< DMatch > auxMatches;
				vector< DMatch > auxBestMatches;
				hAux = homoNoMultiplicated[i+1+j] * hAux;
				auxBestMatches = vecMatch[-j][i+j];
				sort(auxBestMatches.begin(),auxBestMatches.end(),sortByDist);
				auxBestMatches.erase(auxBestMatches.begin()+10,auxBestMatches.end());
				auxBestMatches = removeOutliers(auxBestMatches,i+j,i+1);
				for(int k = 0 ; k < auxBestMatches.size();k++){
					Point2f pt1 = vecKp[i+j][auxBestMatches[k].queryIdx].pt;
					Point2f pt2 = vecKp[i+1][auxBestMatches[k].trainIdx].pt;
					double error = 	projectPointError(pt1,pt2,hAux);
					errorInlier += 	error;
				}
					numMatches+=auxBestMatches.size();
			}
			if(numMatches == 0){
				numMatches ++;
			}
			return (errorInlier/numMatches);
		}

		/*

		*/
		Mat stitchImgs(){
			cout << "\033[1;32m Generando orthomosaico: ... ("<< strImgs.size()-1<< ")\033[0m"<< endl;
			for (int i = 1; i < imgs.size(); i++){
				cout.flush();
				boundBox = stitchWarp(boundBox, imgs[i], H[i])[0];
				string res = "Imagenes/resultados/Pegado/resultados" + to_string(i) + ".png";
				imwrite(res, boundBox);
				cout << "-" << (i+1) * 100 / imgs.size() << "%";
			}
			cout<<endl;
			if(boundBox.channels() < 4){
				boundBox = CommonFunctions::addTransparence(boundBox);
			}
			return boundBox;
		}

		/*
		Utilizo todas las funciones anteriores para realizar el stitching,
		siguiento el siguiente proceso:
		obtengo keypoints y descriptores - los matcheo - obtengo homografias
		- genero boundbox - adapto homografias al tamaño original - pego las imagenes
		*/
		Mat runAll(){
			struct timeval begin;
			gettimeofday(&begin, NULL);

			imgs = CommonFunctions::cargarImagenes(strImgs , tamano);
			begin = CommonFunctions::tiempo(begin, "cargar las imagenes:");

			imgHeight =imgs[0].rows;
			imgWidth =imgs[0].cols;

			detectAndDescript();
			begin = CommonFunctions::tiempo(begin, "obtener keypoints:");

			matchKp();
			begin = CommonFunctions::tiempo(begin, "realizar matching:");

			getHomographies();
			begin = CommonFunctions::tiempo(begin, " obtener las homografias: ");

			// getHomographyError();
			// begin = CommonFunctions::tiempo(begin, " obtener error total: ");

			findBoundBoxLimits();

			if(!evaluateHomography()){
				return Mat();
			}

			generateBoundBox();
			begin = CommonFunctions::tiempo(begin, " obtener boundbox y su homografia: ");

			//adapto los parametros necesarios para que al pegar, se pegue la imagen grande
			rescaleHomographies();

			//USANDO LAS HOMOGRAFIAS, COMIENZO EL PEGADO DE LAS IMAGENES
			stitchImgs();
			begin = CommonFunctions::tiempo(begin, " generar el orthomosaico: ");

			return boundBox;
		}

};

#endif
