#ifndef UAVAgroStateStitcher_H
#define UAVAgroStateStitcher_H

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
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
using namespace cv::xfeatures2d;
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
		int imgHeight;
		int imgWidth;
		int minKeypoints;
		bool usarHomografia;
		vector<int> minMax;
		vector<bool> darVuelta;
		vector<Mat> imgs;
		vector<Mat> borders;
		vector<Mat> vecDesc;
		vector<Mat> H;
		vector<Mat> homoNoMultiplicated;
		vector<string> strImgs;
		vector< vector<KeyPoint> > vecKp;
		vector< vector< DMatch > > vecMatch[3];
		vector< vector< DMatch > > best_inliers;
		Mat boundBox;
		double totalError;
		double alfaBA;

		UAVAgroStateStitcher(vector<string> strImgs,
					vector<int> minMax,
					int tamano = 4,
					int minKeypoints=5000,
					float kPoints = 3,
					bool originalsize=false,
					bool usarHomografia =false
					)
		{
			this->tamano = tamano;
			this->kPoints = kPoints;
			this->originalsize=originalsize;
			this->usarHomografia = usarHomografia;
			this->strImgs = strImgs;
			this->minKeypoints = minKeypoints;
			this->minMax = minMax;
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
			
			Mat imgMaskFrame;
			vector<Mat> bgra;
			split(objWarped, bgra);
			imgMaskFrame = bgra[3].clone();
			// for(int i = 0 ; i < 7; i ++){
			// 	blur(imgMaskFrame,imgMaskFrame,Size(100,100));
			// 	Mat newImg;
			// 	imgMaskFrame.copyTo(newImg,bgra[3]);	
			// 	imgMaskFrame = newImg;
			// }
			


			if(obj.channels() == 4){
				//en el caso de que haya transparencia, se hace un pegado especial
				Mat objAux(scene.size(), scene.type(),Scalar(0,0,0,0));
				objWarped.copyTo(objAux, imgMaskWarped);
				scene = copyToTransparent(objAux, scene,imgMaskFrame);
			}else{
				Mat objAux(scene.size(), scene.type(),Scalar(0,0,0));
				// objWarped.copyTo(objAux, imgMaskWarped);
				// scene = specialBlending(objAux, scene,imgMaskFrame);
				objWarped.copyTo(scene, imgMaskWarped);
			}

			return{ scene, imgMaskWarped };
		}
		/*
		divide una imagen en todos sus canales, para poder sobre escribir la
		escena, solo en el caso de que el objeto no sea transparente en esa parte
		*/
		Mat copyToTransparent(Mat obj, Mat scene, Mat mask){
			mask.convertTo(mask,CV_32F);
			mask = mask / 255;
			
			Mat rgbaObj[4];
			split(obj,rgbaObj);
			for(int i=0;i < obj.rows;i++){
				for(int j=0;j < obj.cols;j++){
					if(rgbaObj[3].at<uchar>(i,j) == 255 ){
						if(scene.at<Vec4b>(i,j) != Vec4b(0,0,0,0)){
							scene.at<Vec4b>(i,j)[0] = obj.at<Vec4b>(i,j)[0] * mask.at<float>(i,j) + scene.at<Vec4b>(i,j)[0] * (1 - mask.at<float>(i,j));
							scene.at<Vec4b>(i,j)[1] = obj.at<Vec4b>(i,j)[1] * mask.at<float>(i,j) + scene.at<Vec4b>(i,j)[1] * (1 - mask.at<float>(i,j));
							scene.at<Vec4b>(i,j)[2] = obj.at<Vec4b>(i,j)[2] * mask.at<float>(i,j) + scene.at<Vec4b>(i,j)[2] * (1 - mask.at<float>(i,j));
							scene.at<Vec4b>(i,j)[3] = obj.at<Vec4b>(i,j)[3];					
						}else{
							scene.at<Vec4b>(i,j) = obj.at<Vec4b>(i,j);
						}
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
		Mat specialBlending(Mat obj, Mat scene, Mat mask){
			for(int i=0;i < obj.rows;i++){
				for(int j=0;j < obj.cols;j++){
					if(obj.at<Vec3b>(i,j) != Vec3b(0,0,0)){
						if(scene.at<Vec3b>(i,j) != Vec3b(0,0,0) && mask.at<uchar>(i,j) != 0){
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


		double compareMats(int numHomo, Mat homoMatrix){
			Mat objAux;
			Mat grayObj,grayScene;
			warpPerspective(imgs[numHomo+1], objAux, homoMatrix, Size(imgs[numHomo].cols, imgs[numHomo].rows));
			cvtColor(objAux,grayObj,CV_BGR2GRAY);
			cvtColor(imgs[numHomo],grayScene,CV_BGR2GRAY);
			// GaussianBlur( grayObj, grayObj, Size( 7,7 ), 0, 0 );
			// GaussianBlur( grayScene, grayScene, Size( 7,7 ), 0, 0 );
			Mat newBorder = grayObj;
			double white=0;
			double cantPx = 0;
			for(int i = 0;i < newBorder.rows;i++){
				for(int j = 0 ; j < newBorder.cols;j++){
					if(newBorder.at<uchar>(i,j)){
						white += abs(grayObj.at<uchar>(i,j) - grayScene.at<uchar>(i,j));
						cantPx++;
					}
				}
			}
			double asd = white/cantPx/10;

			return asd;
		}

		double compareMatsBorders(int numHomo, Mat homoMatrix){
			Mat objAux;
			warpPerspective(borders[numHomo+1], objAux, homoMatrix, Size(borders[numHomo].cols, borders[numHomo].rows));
			Mat imgMask = cv::Mat(borders[numHomo+1].size(), CV_8UC1, cv::Scalar(255));
			warpPerspective(imgMask, imgMask, homoMatrix, Size(objAux.cols, objAux.rows));
			Mat newBorder = abs(objAux - borders[numHomo]);
			double white=0;
			double cantPx = 0;
			for(int i = 0;i < newBorder.rows;i++){
				for(int j = 0 ; j < newBorder.cols;j++){
					if(imgMask.at<uchar>(i,j)){
						if(newBorder.at<uchar>(i,j)){
							white++;
						}
							cantPx++;
					}
				}
			}
			double asd = white/cantPx;

			return asd;
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
					float kTres = .00001;
					vector<KeyPoint> keypoints;
					Mat descriptors;
					int cantKp = -1;
					while(keypoints.size() < minKeypoints && cantKp!=keypoints.size()){	
						cantKp = keypoints.size();				
						// Ptr<cv::BRISK> orb = cv::BRISK::create(kTres);
						Ptr<AKAZE> orb = cv::AKAZE::create(
							AKAZE::DESCRIPTOR_MLDB,0,3, kTres);
						Mat mask = Mat();
						Mat tmp;

						if(imgs[i].channels() == 4){
							cvtColor(imgs[i], tmp, CV_BGRA2GRAY);
							threshold(tmp, mask, 1, 255, THRESH_BINARY);
							Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
							erode(mask, mask, kernel, Point(1, 1), 20);
						}
						orb->detectAndCompute(imgs[i] , mask , keypoints , descriptors);
						kTres/=2;
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
			int vecKpSize = vecKp.size();
			parallel_for_(Range(0, vecDesc.size()-1), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					// cout << "Empezo Match "+to_string(i)+ ". \n";
					vector< DMatch > matches;
					matcher.match(vecDesc[i],vecDesc[i+1], vecMatch[0][i]);
					
					if( i < (vecKpSize-2)  ){
						matcher.match(vecDesc[i],vecDesc[i+2], vecMatch[1][i]);
					}
					if( i < (vecKpSize-3) ){
						matcher.match(vecDesc[i],vecDesc[i+3], vecMatch[2][i]);
					}

					cout << "Termino Match "+to_string(i)+ " con "+ to_string(vecMatch[0][i].size()) +".\n" ;
				}
			});
		}
		/*
		elimino matches 'erroneos' usando como criterio para saber si son malos
		o buenos el hecho de que entre cada imagen hay un desplazamiento
		*/
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

		Mat findHomoAndInliers(vector<Point2f> scene, vector<Point2f> obj,
						vector< DMatch > best_matches, vector< DMatch > &inliers){
			Mat maskinliers;
			Mat hAux = findHomography(scene,obj,CV_RANSAC,3,maskinliers);
			for(int i = 0 ; i < best_matches.size();i++){
				if(maskinliers.at<uchar>(0,i)){
					inliers.push_back(best_matches[i]);
				}
			}
			return hAux;
		}
		/*
		elimino matches 'erroneos' usando un criterio estadistico
		*/
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
		void getHomography(int numHomo)
		{
			double minHomoX = 9999;
			double minHomoY=9999;
			double minError=9999;
			int vueltasI= ((vecMatch[0][numHomo].size()>1000)? 1000 : vecMatch[0][numHomo].size());
			int bestvalx;
			vector< DMatch > best_matches;
			int minMatches = 3;
			vector<double> auxHomoX(vueltasI);
			vector<double> auxHomoY(vueltasI);
			vector<double> auxError(vueltasI);

			sort(vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].end(),sortByDist);

			parallel_for_(Range(minMatches, vueltasI), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					vector< DMatch > goodm;
					goodm.insert(goodm.end(),vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].begin()+i);
					goodm = removeOutliers(goodm, numHomo,numHomo+1);				
					Mat auxH;
					if(goodm.size() >= 3){
						vector<Point2f> obj;
						vector<Point2f> scene;
						for (int l = 0; l < goodm.size(); l++) {
							obj.push_back(vecKp[numHomo][goodm[l].queryIdx].pt);
							scene.push_back(vecKp[numHomo+1][goodm[l].trainIdx].pt);
						}

						if(usarHomografia){
							auxH = findHomography(scene,obj,CV_RANSAC);
						}else{
							auxH = rigidToHomography( estimateRigidTransform(scene,obj,false) );
						}
					}
					if(auxH.dims != 0){
						auxError[i] = compareMats(numHomo,auxH);
						Mat prodH=H[numHomo]*auxH;
						// por trigonometria aplico lo siguiente
						auxHomoX[i] = abs( pow(prodH.at<double>(0,0),2) + pow(prodH.at<double>(0,1),2) -1);
						auxHomoY[i] = abs( pow(prodH.at<double>(1,0),2) + pow(prodH.at<double>(1,1),2) -1);
					}else{
						auxError[i] = 9999;
						auxHomoY[i] = 9999;	
						auxHomoX[i] = 9999;	
					}
				}
			});
			double min=10;
			double max=0;
			for(int i = minMatches ; i < auxHomoX.size();i++){
				if(auxHomoX[i] != 9999){
					if(auxHomoX[i] > max){
						max = auxHomoX[i];
					}
				}
				if(auxHomoX[i] < min){
						min = auxHomoX[i];
				}
			}
			for(int i = minMatches ; i < auxHomoX.size();i++){
				auxHomoX[i] = (auxHomoX[i]-min)  / (max-min);
			}
			min=10; max = 0;
			for(int i = minMatches ; i < auxHomoY.size();i++){
				if(auxHomoY[i] != 9999){
					if(auxHomoY[i] > max){
						max = auxHomoY[i];
					}
				}
				if(auxHomoY[i] < min){
						min = auxHomoY[i];
				}
			}
			for(int i = minMatches ; i < auxHomoY.size();i++){
				auxHomoY[i] = (auxHomoY[i]-min)  / (max-min);
			}
			min=10; max = 0;
			for(int i = minMatches ; i < auxError.size();i++){
				if(auxError[i] != 9999){
					if(auxError[i] > max){
						max = auxError[i];
					}
				}
				if(auxError[i] < min){
						min = auxError[i];
				}
			}
			for(int i = minMatches ; i < auxError.size();i++){
				auxError[i] = (auxError[i]-min)  / (max-min);
			}


			for(int i=minMatches;i<vueltasI;i++){
				if( (auxHomoX[i] + auxHomoY[i] + 1.5*auxError[i]) < minError){
					bestvalx = i;
					minHomoX = auxHomoX[i];
					minHomoY = auxHomoY[i];
					minError = (auxHomoX[i] + auxHomoY[i] + 1.5*auxError[i]);
				}
			}

			best_matches.insert(best_matches.end(),vecMatch[0][numHomo].begin(),vecMatch[0][numHomo].begin()+bestvalx);
			best_matches = removeOutliers(best_matches, numHomo,numHomo+1);
			vector<Point2f> obj;
			vector<Point2f> scene;
			for (int l = 0; l < best_matches.size(); l++) {
				obj.push_back(vecKp[numHomo][best_matches[l].queryIdx].pt);
				scene.push_back(vecKp[numHomo+1][best_matches[l].trainIdx].pt);
			}

			best_inliers[numHomo] = best_matches;
			if(minError == 9999){
				cout << "Homografia vacia: " << numHomo+1 << endl;
				homoNoMultiplicated[numHomo+1] = (Mat::eye(3, 3, CV_64F));
			}else{
				if(usarHomografia){
					best_inliers[numHomo] = vector< DMatch >();
					homoNoMultiplicated[numHomo+1] = findHomoAndInliers(scene, obj, best_matches, best_inliers[numHomo]);
				}else{
					homoNoMultiplicated[numHomo+1] = rigidToHomography( estimateRigidTransform(scene,obj,false) );
				}
			}
			

			Mat aux;
			cout << " " << bestvalx << endl;
			drawMatches(removeAlpha(imgs[numHomo]), vecKp[numHomo],removeAlpha(imgs[numHomo+1]),vecKp[numHomo+1],best_inliers[numHomo],aux,
			Scalar::all(-1),Scalar::all(-1),
			vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/Pegado/output/matches/matchs"+to_string(numHomo)+".png",aux);
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

		void getHomographies(){
			cout << "\033[1;32mGenerando homografias: \033[0m" << endl;
			H = vector<Mat>(imgs.size());
			homoNoMultiplicated = vector<Mat>(imgs.size());
			H[0] = (Mat::eye(3, 3, CV_64F));
			homoNoMultiplicated[0] = (Mat::eye(3, 3, CV_64F));
			alfaBA = 1000;
			for(int i = 0; i < imgs.size()-1;i++){
				cout << "Realizando Homografia "+to_string(i)+ ". \n";
				//caso comun
				getHomography(i);

				H[i+1] = (H[i] * homoNoMultiplicated[i+1]);
				H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
			}
			alfaBA = 2;
			for(int j = 0; j < 0 ;j++){
				for(int i = 0; i < imgs.size()-1;i++){
					cout << "Realizando Homografia "+to_string(i)+ ". \n";
					//caso comun
					homoNoMultiplicated[i+1] = getAfterHomographyError( i,  homoNoMultiplicated[i+1]);
					H[i+1] = (H[i] * homoNoMultiplicated[i+1]);
					H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
				}
			}
		}
		/*
		En base a las homografias, se obtienen los valores que van a delimitar al bound box,
		y evalua que estos no sean de una homogragia mal calculada
		*/
		bool findBoundBoxLimits(){
			cout << "\033[1;32mObteniendo bordes boundbox: \033[0m"<< endl;
			FileStorage fsHomo("Data/Homografias/homografias.yml", FileStorage::WRITE);
			for(int i = 0; i < H.size()-1; i++){
				// CUANDO ALGUNOS PARAMETROS SON MENOS QUE 0 ES PQ ESTA INCLINADA DE MANERA Q FAVORECE A MIN X
				double newMinX = H[i+1].at<double>(0,2) +
				( (H[i+1].at<double>(0,0) < 0)? imgs[i+1].cols  * H[i+1].at<double>(0,0) : 0 ) +
				( (H[i+1].at<double>(0,1) < 0)? imgs[i+1].rows  * H[i+1].at<double>(0,1) : 0 );
				// CUANDO ALGUNOS PARAMETROS SON MENOS QUE 0 ES PQ ESTA INCLINADA DE MANERA Q FAVORECE A MIN Y
				double newMinY = H[i+1].at<double>(1,2) +
				( (H[i+1].at<double>(1,0) < 0)? imgs[i+1].cols  * H[i+1].at<double>(1,0) : 0 ) +
				( (H[i+1].at<double>(1,1) < 0)? imgs[i+1].rows  * H[i+1].at<double>(1,1) : 0 );
				// CUANDO ALGUNOS PARAMETROS SON MENOS QUE 0 ES PQ ESTA DADA VUELTA EN X
				double newMaxX = H[i+1].at<double>(0,2) - imgWidth +
				( (imgWidth  * H[i+1].at<double>(0,0) > 0) ?imgs[i+1].cols  * H[i+1].at<double>(0,0) : 0) +
				( (imgs[i+1].rows  * H[i+1].at<double>(0,1) > 0) ? imgs[i+1].rows  * H[i+1].at<double>(0,1) : 0);
				// CUANDO ALGUNOS PARAMETROS SON MENOS QUE 0 ES PQ ESTA DADA VUELTA EN Y
				double newMaxY = H[i+1].at<double>(1,2) - imgHeight +
				( (imgWidth * H[i+1].at<double>(1,0) > 0) ?imgs[i+1].cols * H[i+1].at<double>(1,0) : 0) +
				( (imgs[i+1].rows  * H[i+1].at<double>(1,1) > 0) ? imgs[i+1].rows  * H[i+1].at<double>(1,1) : 0);
				if(newMinX < xMin){
					xMin = newMinX;
				}
				if(newMaxX > xMax){
					xMax = newMaxX;
				}
				if(newMinY < yMin){
					yMin = newMinY;
				}
				if(newMaxY > yMax){
					yMax = newMaxY;
				}
				fsHomo << "homografia"+to_string(i+1) << H[i+1];
			}
			fsHomo.release();
			xMin -= 20;
			yMin -= 20;
			yMax += 20;
			xMax += 20;
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
			imwrite("res.png", boundBox);
			//adapto los keypoints de la primer imagen, al boundbox generado con esta
			Point2f ptAux(abs(xMin),abs(yMin));
			for(int i=0;i<vecKp[0].size();i++){
				vecKp[0][i].pt+=ptAux;
			}
			//vuelvo a calcular la homografia para la imagen con bordes
			imgs[0] = boundBox;
			vector<Point2f> obj;
			vector<Point2f> scene;
			for (int l = 0; l < best_inliers[0].size(); l++) {
				obj.push_back(vecKp[0][best_inliers[0][l].queryIdx].pt);
				scene.push_back(vecKp[1][best_inliers[0][l].trainIdx].pt);
			}

			if(usarHomografia){
				homoNoMultiplicated[1] = findHomography(scene,obj,CV_RANSAC);
			}else{
				homoNoMultiplicated[1] = rigidToHomography( estimateRigidTransform(scene,obj,false) );
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
				imgs = CommonFunctions::cargarImagenes(strImgs , 1,IMREAD_UNCHANGED);
				removeCorners();
				fsHomo.release();

				boundBox = imgs[0];
				boundBox = CommonFunctions::boundingBox(boundBox, abs(yMin) , yMax , abs(xMin),xMax);
			}
		}

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


		
		
		double* matToCamera(Mat mat){
			double* camera;
			camera = new double[6];
			
			camera[0]=mat.at<double>(0,0);
			camera[1]=mat.at<double>(0,1);
			camera[2]=mat.at<double>(0,2);
			camera[3]=mat.at<double>(1,0);
			camera[4]=mat.at<double>(1,1);
			camera[5]=mat.at<double>(1,2);

			return camera;
		}

		Mat cameraToMat(double* camera, Mat mat){
			mat.at<double>(0,0)=camera[0];
			mat.at<double>(0,1)=camera[1];
			mat.at<double>(0,2)=camera[2];
			mat.at<double>(1,0)=camera[3];
			mat.at<double>(1,1)=camera[4];
			mat.at<double>(1,2)=camera[5];
			return mat;
		}

		struct ActualReprojectionError {
			ActualReprojectionError(double observed_x, double observed_y)
				: observed_x(observed_x), observed_y(observed_y) {}

			template <typename T>
			bool operator()(const T* const camera,
							const T* const point,
							T* residuals) const {

				// camera[0,1,2] are the angle-axis rotation.
				T p[2];
				// ceres::AngleAxisRotatePoint(camera, point, p);
				p[0]=camera[0]* point[0] + camera[1]* point[1] + camera[2];
				p[1]=camera[3]* point[0] + camera[4]* point[1] + camera[5];

				// The error is the difference between the predicted and observed position.
				residuals[0]  = p[0] - T(observed_x);
				residuals[1] = p[1] - T(observed_y);
				
				return true;
			}

			// Factory to hide the construction of the CostFunction object from
			// the client code.
			static ceres::CostFunction* Create(const double observed_x,
												const double observed_y) {
				return (new ceres::AutoDiffCostFunction<ActualReprojectionError, 2,6,2>(
							new ActualReprojectionError(observed_x, observed_y)));
			}

			double observed_x;
			double observed_y;
		};
		
		struct AfterReprojectionError {
			AfterReprojectionError(double observed_x, double observed_y, Mat H)
				: observed_x(observed_x), observed_y(observed_y),H(H) {}

			template <typename T>
			bool operator()(const T* const camera,
							const T* const point,
							T* residuals) const {

				// camera[0,1,2] are the angle-axis rotation.
				T p[2];
				// ceres::AngleAxisRotatePoint(camera, point, p);
				T cameraAux[6];
				cameraAux[0]= camera[0] * T(H.at<double>(0,0)) + camera [1] * T(H.at<double>(1,0)) + camera[2] * T(H.at<double>(2,0));
				cameraAux[1]= camera[0] * T(H.at<double>(0,1)) + camera [1] * T(H.at<double>(1,1)) + camera[2] * T(H.at<double>(2,1));
				cameraAux[2]= camera[0] * T(H.at<double>(0,2)) + camera [1] * T(H.at<double>(1,2)) + camera[2] * T(H.at<double>(2,2));
				cameraAux[3]= camera[3] * T(H.at<double>(0,0)) + camera [4] * T(H.at<double>(1,0)) + camera[5] * T(H.at<double>(2,0));
				cameraAux[4]= camera[3] * T(H.at<double>(0,1)) + camera [4] * T(H.at<double>(1,1)) + camera[5] * T(H.at<double>(2,1));
				cameraAux[5]= camera[3] * T(H.at<double>(0,2)) + camera [4] * T(H.at<double>(1,2)) + camera[5] * T(H.at<double>(2,2));
				
				p[0]=cameraAux[0]* point[0] + cameraAux[1]* point[1] + cameraAux[2];
				p[1]=cameraAux[3]* point[0] + cameraAux[4]* point[1] + cameraAux[5];

				// The error is the difference between the predicted and observed position.
				residuals[0]  = p[0] - T(observed_x);
				residuals[1] = p[1] - T(observed_y);
				if(abs(residuals[1]) > T(30) || abs(residuals[0]) > T(30)){
					residuals[1] = T(0);
					residuals[0] = T(0);
				}
				
				return true;
			}

			// Factory to hide the construction of the CostFunction object from
			// the client code.
			static ceres::CostFunction* Create(const double observed_x,
												const double observed_y,
												const Mat H) {
				return (new ceres::AutoDiffCostFunction<AfterReprojectionError, 2,6,2>(
							new AfterReprojectionError(observed_x, observed_y,H)));
			}

			double observed_x;
			double observed_y;
			Mat H;
		};

		struct BeforeReprojectionError {
			BeforeReprojectionError(double observed_x, double observed_y, Mat H)
				: observed_x(observed_x), observed_y(observed_y),H(H) {}

			template <typename T>
			bool operator()(const T* const camera,
							const T* const point,
							T* residuals) const {

				// camera[0,1,2] are the angle-axis rotation.
				T p[2];
				// ceres::AngleAxisRotatePoint(camera, point, p);
				T cameraAux[6];
				cameraAux[0]= camera[0] * T(H.at<double>(0,0)) + camera [3] * T(H.at<double>(0,1)) + T(0) * T(H.at<double>(0,2));
				cameraAux[1]= camera[1] * T(H.at<double>(0,0)) + camera [4] * T(H.at<double>(0,1)) + T(0) * T(H.at<double>(0,2));
				cameraAux[2]= camera[2] * T(H.at<double>(0,0)) + camera [5] * T(H.at<double>(0,1)) + T(1) * T(H.at<double>(0,2));
				cameraAux[3]= camera[0] * T(H.at<double>(1,0)) + camera [3] * T(H.at<double>(1,1)) + T(0) * T(H.at<double>(1,2));
				cameraAux[4]= camera[1] * T(H.at<double>(1,0)) + camera [4] * T(H.at<double>(1,1)) + T(0) * T(H.at<double>(1,2));
				cameraAux[5]= camera[2] * T(H.at<double>(1,0)) + camera [5] * T(H.at<double>(1,1)) + T(1) * T(H.at<double>(1,2));


				p[0]=cameraAux[0]* point[0] + cameraAux[1]* point[1] + cameraAux[2];
				p[1]=cameraAux[3]* point[0] + cameraAux[4]* point[1] + cameraAux[5];

				// The error is the difference between the predicted and observed position.
				residuals[0]  = p[0] - T(observed_x);
				residuals[1] = p[1] - T(observed_y);
				if(abs(residuals[1]) > T(30) || abs(residuals[0]) > T(30)){
					residuals[1] = T(0);
					residuals[0] = T(0);
				}
				
				return true;
			}

			// Factory to hide the construction of the CostFunction object from
			// the client code.
			static ceres::CostFunction* Create(const double observed_x,
												const double observed_y,
												const Mat H) {
				return (new ceres::AutoDiffCostFunction<BeforeReprojectionError, 2,6,2>(
							new BeforeReprojectionError(observed_x, observed_y,H)));
			}

			double observed_x;
			double observed_y;
			Mat H;
		};
		
		Mat getAfterHomographyError(int i, Mat homo){
			int maxMatches = 2;
			int afterMatches = ((imgs.size()-1 - i) > (maxMatches+1))? (maxMatches+1) : imgs.size()-1 - i ;
			int beforeMatches = (i < maxMatches)? i : maxMatches;

			Problem problem;

			double* camera = matToCamera(homo.clone());
			Mat hMult = (Mat::eye(3, 3, CV_64F));

			for(int k = 0 ; k < best_inliers[i].size();k++){
				Point2f pt1 = vecKp[i][best_inliers[i][k].queryIdx].pt;
				Point2f pt2 = vecKp[i+1][best_inliers[i][k].trainIdx].pt;
				ceres::CostFunction* cost_function =
				ActualReprojectionError::Create(pt1.x,pt1.y);
				double* point = new double[2];
				point[0] = pt2.x;
				point[1] = pt2.y;
				problem.AddResidualBlock(cost_function, NULL,camera,point);
			}

			for(int j = 1; j < afterMatches; j++){
				hMult *= homoNoMultiplicated[i+1+j];
				vector< DMatch > auxMatches;
				vector< DMatch > auxBestMatches;
				auxMatches = vecMatch[j][i];
				sort(auxMatches.begin(),auxMatches.end(),sortByDist);
				auxMatches.erase(auxMatches.begin()+30,auxMatches.end());
				auxMatches = removeOutliers(auxMatches,i,i+1+j);
				auxBestMatches = auxMatches;
				for(int k = 0 ; k < auxBestMatches.size();k++){
					Point2f pt1 = vecKp[i][auxBestMatches[k].queryIdx].pt;
					Point2f pt2 = vecKp[i+1+j][auxBestMatches[k].trainIdx].pt;
					double* point = new double[2];
					point[0] = pt2.x;
					point[1] = pt2.y;
					ceres::CostFunction* cost_function =
						AfterReprojectionError::Create(pt1.x,pt1.y,hMult.clone());
					problem.AddResidualBlock(cost_function, NULL,camera,point);
				}
			}


			hMult = (Mat::eye(3, 3, CV_64F));

			for(int j = -1; j > -(beforeMatches+1) ; j--){
				vector< DMatch > auxMatches;
				vector< DMatch > auxBestMatches;
				hMult = homoNoMultiplicated[i+1+j] * hMult;
				auxMatches = vecMatch[-j][i+j];
				sort(auxMatches.begin(),auxMatches.end(),sortByDist);
				auxMatches.erase(auxMatches.begin()+30,auxMatches.end());
				auxMatches = removeOutliers(auxMatches,i+j,i+1);
				auxBestMatches = auxMatches;
				for(int k = 0 ; k < auxBestMatches.size();k++){
					Point2f pt1 = vecKp[i+j][auxBestMatches[k].queryIdx].pt;
					Point2f pt2 = vecKp[i+1][auxBestMatches[k].trainIdx].pt;
					double* point = new double[2];
					point[0] = pt2.x;
					point[1] = pt2.y;
					ceres::CostFunction* cost_function =
						BeforeReprojectionError::Create(pt1.x,pt1.y,hMult.clone());
					problem.AddResidualBlock(cost_function, NULL,camera,point);
				}
			}

			ceres::Solver::Options options;
			options.linear_solver_type = ceres::DENSE_QR;
			// options.function_tolerance = 1e-500;
			// options.parameter_tolerance = 1e-500;
			// options.minimizer_progress_to_stdout = true;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			cout << summary.BriefReport() << "\n"<<"\n";
			Mat hAux = cameraToMat(camera,homo.clone());

			return hAux;
		}


		/*

		*/
		Mat stitchImgs(){
			cout << "\033[1;32m Generando orthomosaico: ... ("<< strImgs.size()-1<< ")\033[0m"<< endl;

			for (int i = 1; i < imgs.size(); i++){
				cout.flush();
				boundBox = stitchWarp(boundBox, imgs[i], H[i])[0];
				// string res = "Imagenes/Pegado/output/resultados" + to_string(i) + ".png";
				// imwrite(res, boundBox);
				cout << "-" << (i+1) * 100 / imgs.size() << "%";
			}
			cout<<endl;
			if(boundBox.channels() < 4){
				boundBox = CommonFunctions::addTransparence(boundBox);
			}
			return boundBox;
		}
		

		Mat multiBandStitch(){
			cout << "\033[1;32m Generando orthomosaico: ... ("<< strImgs.size()-1<< ")\033[0m"<< endl;
			detail::MultiBandBlender blender(false,2);
			cvtColor(boundBox, boundBox, cv::COLOR_BGRA2BGR);			
			blender.prepare(Rect(0,0,boundBox.cols,boundBox.rows));

			Mat bigImage, mask;
			for (int i = 1; i < imgs.size(); i++){
				Mat  objWarped, imgMaskWarped, imgMask = cv::Mat(imgs[i].size(), CV_8UC1, cv::Scalar(255));
				warpPerspective(imgMask, imgMaskWarped, H[i], Size(boundBox.cols, boundBox.rows));
				Mat obj;
				cvtColor(imgs[i], obj, cv::COLOR_BGRA2BGR);
				warpPerspective(obj, objWarped, H[i], Size(boundBox.cols, boundBox.rows));
				// CommonFunctions::showWindowNormal(objWarped);

				Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
				erode(imgMaskWarped, imgMaskWarped, kernel, Point(1, 1), 5);
				

				blender.feed(objWarped.clone(), imgMaskWarped, Point(0, 0));
				cout << "-" << (i+1) * 100 / imgs.size() << "%";
			}
			blender.blend(bigImage, mask);
			bigImage.convertTo(bigImage, (bigImage.type() / 8) * 8);
			// CommonFunctions::showWindowNormal(bigImage);
			cout<<endl;
			if(boundBox.channels() < 4){
				boundBox = CommonFunctions::addTransparence(bigImage);
			}
			return boundBox;
		}

		/*
		quito las esquinas para remover el vignetting
		*/
		void removeCorners(){
			for(int i = 0 ; i < imgs.size();i++){
				int sizeCorner = imgs[i].cols/6;
				for(int j= 0 ; j < sizeCorner ; j++){
					for(int k = 0 ;k < sizeCorner-j;k++){
						imgs[i].at<Vec4b>(j,k)[3] = 0;
					}
				}
				for(int j= 0 ; j < sizeCorner ; j++){
					for(int k = imgs[i].cols-sizeCorner+j;k<imgs[i].cols;k++){
						imgs[i].at<Vec4b>(j,k)[3] = 0;
					}
				}
				for(int j= imgs[i].rows-sizeCorner ; j < imgs[i].rows; j++){
					for(int k = 0 ;k < j-(imgs[i].rows-sizeCorner);k++){
						imgs[i].at<Vec4b>(j,k)[3] = 0;
					}
				}
				for(int j= imgs[i].rows-sizeCorner ; j < imgs[i].rows; j++){
					for(int k = imgs[i].cols-(j-(imgs[i].rows-sizeCorner));
					k<imgs[i].cols;
					k++){
						imgs[i].at<Vec4b>(j,k)[3] = 0;
					}
				}
			}
		}


		void compensateBright(){
						float mediaDeMediasB=0;
			float mediaDeMediasG=0;
			float mediaDeMediasR=0;
			vector<float> vecMediaB(imgs.size());
			vector<float> vecMediaG(imgs.size());
			vector<float> vecMediaR(imgs.size());
			for(int i = 0; i < imgs.size(); i++){
				vecMediaB[i]=0;
				vecMediaG[i]=0;
				vecMediaR[i]=0;
				int cantPix=0;
				for(int j = 0; j < imgs[i].rows ;j++){
					for(int k = 0; k < imgs[i].cols; k++){
						if(imgs[i].at<Vec4b>(j,k)[3] != 0){
							vecMediaB[i] += imgs[i].at<Vec4b>(j,k)[0];
							vecMediaG[i] += imgs[i].at<Vec4b>(j,k)[1];
							vecMediaR[i] += imgs[i].at<Vec4b>(j,k)[2];
							cantPix++;
						}
					}
				}
				vecMediaB[i]/=cantPix;
				vecMediaG[i]/=cantPix;
				vecMediaR[i]/=cantPix;
				mediaDeMediasB+=vecMediaB[i];
				mediaDeMediasG+=vecMediaG[i];
				mediaDeMediasR+=vecMediaR[i];
				cout << "Media B: "<< vecMediaB[i] <<" G "<< vecMediaG[i]  <<" R "<< vecMediaR[i] << endl;
			}
			mediaDeMediasB/=imgs.size();
			mediaDeMediasG/=imgs.size();
			mediaDeMediasR/=imgs.size();
			cout << mediaDeMediasB << " g " << mediaDeMediasG << "r" << mediaDeMediasR << endl; 
			for(int i = 0; i < imgs.size(); i++){
				float difB = vecMediaB[i] - mediaDeMediasB;
				float difG = vecMediaG[i] - mediaDeMediasG;
				float difR = vecMediaR[i] - mediaDeMediasR;
				cout << difB << " g " << difG << "r" << difR << endl; 
				vecMediaB[i]=0;
				vecMediaG[i]=0;
				vecMediaR[i]=0;
				int cantPix=0;
				for(int j = 0; j < imgs[i].rows ;j++){
					for(int k = 0; k < imgs[i].cols; k++){
						if(imgs[i].at<Vec4b>(j,k)[0] != 0){
							imgs[i].at<Vec4b>(j,k)[0] -= difB;
							imgs[i].at<Vec4b>(j,k)[1] -= difG;
							imgs[i].at<Vec4b>(j,k)[2] -= difR;
							vecMediaB[i] += imgs[i].at<Vec4b>(j,k)[0];
							vecMediaG[i] += imgs[i].at<Vec4b>(j,k)[1];
							vecMediaR[i] += imgs[i].at<Vec4b>(j,k)[2];
							cantPix++;
						}
					}
				}
				vecMediaB[i]/=cantPix;
				vecMediaG[i]/=cantPix;
				vecMediaR[i]/=cantPix;
				cout << "Media B: "<< vecMediaB[i] <<" G "<< vecMediaG[i]  <<" R "<< vecMediaR[i] << endl;
			}
		}
		/*
		Utilizo todas las funciones anteriores para realizar el stitching,
		siguiento el siguiente proceso:
		obtengo keypoints y descriptores - los matcheo - obtengo homografias
		- genero boundbox - adapto homografias al tamaño original - pego las imagenes
		*/
		Mat runAll(){			
			// usarHomografia = true;
			struct timeval begin;
			gettimeofday(&begin, NULL);

			imgs = CommonFunctions::cargarImagenes(strImgs , tamano,IMREAD_UNCHANGED);
			// compensateBright();
			// borders = CommonFunctions::getBorders(imgs,minMax);
			
			// for(int i = 0 ; i < imgs.size() ; i++){
			// 	vector<Mat> RGB;
			// 	split(imgs[i],RGB);
			// 	RGB[1] = Mat::zeros(imgs[i].rows, imgs[i].cols, CV_8UC1);
			// 	merge(RGB,imgs[i]);
			// 	// CommonFunctions::showWindowNormal(imgs[i]);
			// }

			removeCorners();

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

			// imgs = CommonFunctions::cargarImagenes(strImgs , tamano, IMREAD_UNCHANGED);

			generateBoundBox();
			begin = CommonFunctions::tiempo(begin, " obtener boundbox y su homografia: ");

			//adapto los parametros necesarios para que al pegar, se pegue la imagen grande
			rescaleHomographies();

			//USANDO LAS HOMOGRAFIAS, COMIENZO EL PEGADO DE LAS IMAGENES
			stitchImgs();
			// multiBandStitch();
			begin = CommonFunctions::tiempo(begin, " generar el orthomosaico: ");

			return boundBox;
		}

};

#endif
