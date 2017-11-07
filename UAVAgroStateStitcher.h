#ifndef UAVAgroStateStitcher_H
#define UAVAgroStateStitcher_H

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
		vector<Mat> imgs;
		vector<string> strImgs;
		vector<Mat> vecDesc;
		vector< vector<KeyPoint> > vecKp;
		vector< vector< DMatch > > vecMatch;
		vector< vector< DMatch > > best_inliers;
		vector<Mat> H;
		vector<Mat> homoNoMultiplicated;
		Mat boundBox;
		
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


		Mat compareMats(Mat scene, Mat obj, Mat homoMatrix){
			Mat objAux,tmp;
			warpPerspective(obj, objAux, homoMatrix, Size(scene.cols, scene.rows));
			threshold(objAux, tmp, 1, 255, THRESH_BINARY);
			double error=0;
			double cantError=0;
			if(scene.channels() == 4){
				for(int i=0;i < objAux.rows;i++){
					for(int j=0;j < objAux.cols;j++){
						if(objAux.at<Vec4b>(i,j) != Vec4b(0,0,0,0) && scene.at<Vec4b>(i,j) != Vec4b(0,0,0,0)){
							error += abs( scene.at<Vec3b>(i,j)[0] - objAux.at<Vec3b>(i,j)[0] );
							error += abs( scene.at<Vec3b>(i,j)[1] - objAux.at<Vec3b>(i,j)[1] );
							error += abs( scene.at<Vec3b>(i,j)[2] - objAux.at<Vec3b>(i,j)[2] );
							cantError++;
						}
					}
				}
			}else{
				for(int i=0;i < obj.rows;i++){
					for(int j=0;j < obj.cols;j++){
						if(obj.at<Vec3b>(i,j) != Vec3b(0,0,0) && scene.at<Vec3b>(i,j) != Vec3b(0,0,0)){
	
						}
					}
				}
			}
			// objAux = abs(scene - objAux);
			// CommonFunctions::showWindowNormal(objAux);
			cout<< "eerror: " << error / cantError << endl;
							
			return objAux;
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
			vecMatch = vector< vector< DMatch > >(imgs.size()); 
			best_inliers = vector< vector< DMatch > >(imgs.size());
			BFMatcher matcher(NORM_HAMMING,true);
			parallel_for_(Range(0, vecDesc.size()-1), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					// cout << "Empezo Match "+to_string(i)+ ". \n";
					vector< DMatch > matches;
					matcher.match(vecDesc[i],vecDesc[i+1], matches);
					vecMatch[i] = matches;
					cout << "Termino Match "+to_string(i)+ " con "+ to_string(vecMatch[i].size()) + " matches.\n";	
				}
			});
		}
		/*
		elimino matches 'erroneos' usando como criterio para saber si son malos
		o buenos el hecho de que entre cada imagen hay un desplazamiento
		*/
		vector< DMatch > goodMatches(int numMatch,double porcMinY,double porcMaxY,int minvalx, int maxvalx){
			vector< DMatch > gm;
			bool imgMismaAltura = false;
			int minvaly = imgHeight*porcMinY;
			int maxvaly = imgHeight*porcMaxY;
			// int minvalx = imgWidth*porcMinX;
			// int maxvalx = imgWidth*porcMaxX;
			while(gm.size() < 4){
				gm= vector< DMatch >();
				for(int i = 0; i < vecMatch[numMatch].size(); i++){
					Point2f pt0 = vecKp[numMatch][vecMatch[numMatch][i].queryIdx].pt;
					Point2f pt1 = vecKp[numMatch+1][vecMatch[numMatch][i].trainIdx].pt;
					if( (pt1.y-pt0.y) > minvaly && (pt1.y-pt0.y) < maxvaly 
						&& abs(pt1.x-pt0.x) >= minvalx && abs(pt1.x-pt0.x) < maxvalx
						){
						gm.push_back(vecMatch[numMatch][i]);
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
				sort(vecMatch[numMatch].begin(),vecMatch[numMatch].end(),sortByDist);
				gm.insert(gm.end(),vecMatch[numMatch].begin(),vecMatch[numMatch].begin()+50);
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

		/*
		obtengo varias homografias modificando ciertos parametros, y elijo la que
		sea mas adecuada
		*/
		void getHomography(int numHomo)
		{
			double minHomoX = 9999;
			double minHomoY=9999;
			double porcMinY = 0.05;
			double porcMaxY = 0.5;
			int minvalx = 0;
			int maxvalx = 60;
			int bestvalx;double bestPorcMinY;double bestPorcMaxY;
			int vueltasI=10;int vueltasJ=10;int vueltasK=10;
			vector< DMatch > best_matches;
			Mat MaskInliers;
			
			
			for(int i=0;i<vueltasI;i++){
				porcMinY = 0.00;
				for(int j=0;j<vueltasJ;j++){
					maxvalx = 50;
					vector<Mat> auxH(vueltasK);
					vector<Mat> mask_inliers(vueltasK);
					vector< vector< DMatch > > good_matches(vueltasK);
					parallel_for_(Range(0, vueltasK), [&](const Range& range){
						for(int k = range.start;k < range.end ; k++){
							maxvalx = 50 - k;
							good_matches[k] = goodMatches(numHomo,porcMinY,porcMaxY,minvalx,maxvalx);
							// sort(good_matches[k].begin(),good_matches[k].end(),sortByDist);
							///OBTENGO LOS PUNTOS EN LOS QUE SE ENCUENTRAN LOS GOOD MATCHES
							std::vector<Point2f> obj;
							std::vector<Point2f> scene;
							for (int l = 0; l < good_matches[k].size(); l++) {
								obj.push_back(vecKp[numHomo][good_matches[k][l].queryIdx].pt);
								scene.push_back(vecKp[numHomo+1][good_matches[k][l].trainIdx].pt);
							}
							// ARMO LA MATRIZ DE HOMOGRAFIA EN BASE A LOS PUNTOS ANTERIORES
							Mat maskH;
							if(good_matches[k].size() > 4){
								auxH[k] = rigidToHomography( estimateRigidTransform(scene,obj,false) );
								// auxH[k] = findHomography(scene, obj, CV_RANSAC,3,mask_inliers[k]);
							}
						}});
					for(int k=0;k<vueltasK;k++){
						if(!auxH[k].empty()){
							Mat prodH=H[numHomo]*auxH[k];
							// por trigonometria aplico lo siguiente
							double auxHomoX = abs( .9*pow(prodH.at<double>(0,0),2) + .1*pow(prodH.at<double>(0,1),2) -1);
							double auxHomoY = abs( .1*pow(prodH.at<double>(1,0),2) + .9*pow(prodH.at<double>(1,1),2) -1);
							
							if(auxHomoX < minHomoX && auxHomoY < minHomoY){
								homoNoMultiplicated[numHomo+1] = auxH[k];
								minHomoX = auxHomoX;
								minHomoY = auxHomoY;
								bestvalx = maxvalx;
								bestPorcMinY = porcMinY;
								bestPorcMaxY = porcMaxY;
								best_matches = good_matches[k];
								MaskInliers = mask_inliers[k];
							}
						}
					}
					porcMinY += 0.01;
				}
				porcMaxY -= 0.01;
			}
			// compareMats(imgs[numHomo],imgs[numHomo+1],homoNoMultiplicated[numHomo+1]);
			if(minHomoX == 9999){
				homoNoMultiplicated[numHomo+1] = (Mat::eye(3, 3, CV_64F));
			}
			Mat aux;
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
			
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(numHomo)+"inliers.png",aux);
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
			vector< DMatch > good_matches;
			sort(vecMatch[numHomo].begin(),vecMatch[numHomo].end(),sortByDist);
			if(vertical){
				for(int i = 0 ; i < vecMatch[numHomo].size(); i++){
					int y1 = vecKp[numHomo][vecMatch[numHomo][i].queryIdx].pt.y;	
					int y2 = vecKp[numHomo+1][vecMatch[numHomo][i].trainIdx].pt.y;
						if(y1 < (imgs[numHomo].rows/4) && y2 > (imgs[numHomo+1].rows-imgs[numHomo+1].rows/4)){
							good_matches.push_back(vecMatch[numHomo][i]);
						}
				}
			}else{
				for(int i = 0 ; i < vecMatch[numHomo].size(); i++){
					int x1 = vecKp[numHomo][vecMatch[numHomo][i].queryIdx].pt.x;	
					int x2 = vecKp[numHomo+1][vecMatch[numHomo][i].trainIdx].pt.x;
					if(x1 > (imgs[numHomo].cols*3/4) && x2 < (imgs[numHomo+1].cols*3/4)){
						good_matches.push_back(vecMatch[numHomo][i]);
					}
				}
			}
			// good_matches = goodMatches(match,keypoints,.0,.2,0,1000);
			good_matches.erase(good_matches.begin()+100,good_matches.end());

			///OBTENGO LOS PUNTOS EN LOS QUE SE ENCUENTRAN LOS GOOD MATCHES
			std::vector<Point2f> obj;
			std::vector<Point2f> scene;
			for (int l = 0; l < good_matches.size(); l++) {
				obj.push_back(vecKp[numHomo][good_matches[l].queryIdx].pt);
				scene.push_back(vecKp[numHomo+1][good_matches[l].trainIdx].pt);
			}
			// ARMO LA MATRIZ DE HOMOGRAFIA EN BASE A LOS PUNTOS ANTERIORES
			Mat MaskInliers;
			homoNoMultiplicated[numHomo+1] = findHomography(scene, obj, CV_RANSAC,3,MaskInliers);

			compareMats(imgs[numHomo], imgs[numHomo+1],homoNoMultiplicated[numHomo+1]);

			sort(good_matches.begin(),good_matches.end(),sortByDist);
			DMatch lastMatch;
			for(int i = 0 ; i < good_matches.size();i++){
				if(MaskInliers.at<uchar>(0,i)){
					if(lastMatch.queryIdx != good_matches[i].queryIdx){
						best_inliers[numHomo].push_back(good_matches[i]);
						lastMatch = good_matches[i];
					}
				}
			}

			Mat aux;
			drawMatches(removeAlpha(imgs[numHomo]), vecKp[numHomo],removeAlpha(imgs[numHomo+1]),vecKp[numHomo+1],good_matches,aux,
			Scalar::all(-1),Scalar::all(-1),
			std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(numHomo)+".png",aux);
			drawMatches(removeAlpha(imgs[numHomo]), vecKp[numHomo],removeAlpha(imgs[numHomo+1]),vecKp[numHomo+1],best_inliers[numHomo],aux,
			Scalar::all(-1),Scalar::all(-1),
			std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(numHomo)+"inliers.png",aux);
		}

		void getHomographies(){
			cout << "\033[1;32mGenerando homografias: \033[0m" << endl;
			H = vector<Mat>(imgs.size());
			homoNoMultiplicated = vector<Mat>(imgs.size());
			H[0] = (Mat::eye(3, 3, CV_64F));
			homoNoMultiplicated[0] = (Mat::eye(3, 3, CV_64F));
			for(int i = 0; i < imgs.size()-1;i++){
				cout << "Realizando Homografia "+to_string(i)+ ". \n";
				switch(tipoHomografia){
					case 1:{
						//caso comun
						getHomography(i);
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
		}
		/*
		En base a las homografias, se obtienen los valores que van a delimitar al bound box, 
		y evalua que estos no sean de una homogragia mal calculada
		*/
		bool findBoundBoxLimits(){
			cout << "\033[1;32mObteniendo bordes boundbox: \033[0m"<< endl;
			FileStorage fsHomo("Data/Homografias/homografias.yml", FileStorage::WRITE);
			for(int i = 0; i < H.size()-1; i++){
				if(H[i+1].at<double>(0,2) < xMin){
					xMin = H[i+1].at<double>(0,2);
				}
				if(H[i+1].at<double>(0,2) > xMax){
					xMax = H[i+1].at<double>(0,2);
				}
				if(H[i+1].at<double>(1,2) < yMin){
					yMin = H[i+1].at<double>(1,2);
				}
				if(H[i+1].at<double>(1,2) > yMax){
					yMax = H[i+1].at<double>(1,2);
				}
				fsHomo << "homografia"+to_string(i+1) << H[i+1];
			}
			fsHomo.release();
			// yMin -= 1000;
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
			

			homoNoMultiplicated[1] = rigidToHomography( estimateRigidTransform(scene,obj,false) );

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

		double projectPointError(Point2f pt1 , Point2f pt2, Mat hAux){
			Mat X1 = (Mat_<double>(3,1) << pt1.x, pt1.y, 1);
			Mat X2 = (Mat_<double>(3,1) << pt2.x, pt2.y, 1);
			double x = hAux.at<double>(0,0)*X2.at<double>(0,0)+
			hAux.at<double>(0,1)*X2.at<double>(0,1)+
			hAux.at<double>(0,2)*X2.at<double>(0,2);
			double y = hAux.at<double>(1,0)*X2.at<double>(0,0)+
			hAux.at<double>(1,1)*X2.at<double>(0,1)+
			hAux.at<double>(1,2)*X2.at<double>(0,2);
			double z = hAux.at<double>(2,0)*X2.at<double>(0,0)+
			hAux.at<double>(2,1)*X2.at<double>(0,1)+
			hAux.at<double>(2,2)*X2.at<double>(0,2);
			x/=z;
			y/=z;
			double r = abs(x - X1.at<double>(0,0)) + abs(y - X1.at<double>(0,1));
			// int alfa = 100;
			// if(r < alfa){
			// 	r*=r;
			// }else{
			// 	r = 2*alfa*r - alfa*alfa;
			// }
			return r;
		}

		void getHomographyError(){
			cout << "\033[1;32m Obteniendo error total: ... ("<< strImgs.size()-1<< ")\033[0m"<< endl;
			double errorTotal = 0;
			for(int i = 0 ; i < imgs.size()-1;i++){
				double errorInlier = 0;
				int maxMatches = 2;
				int beforeMatches = (i < maxMatches)? i : maxMatches;
				int afterMatches = ((imgs.size()-1) - i > maxMatches)? maxMatches : (imgs.size()-1) - i ;

				Mat hAux = homoNoMultiplicated[i+1].clone(); // eye matrix
				// j == 0
				for(int k = 0 ; k < best_inliers[i].size();k++){
					Point2f pt1 = vecKp[i][best_inliers[i][k].queryIdx].pt;
					Point2f pt2 = vecKp[i+1][best_inliers[i][k].trainIdx].pt;
					errorInlier += 	projectPointError(pt1,pt2,hAux);
				}
				for(int j = 1; j < afterMatches; j++){
					hAux *= homoNoMultiplicated[i+1+j];
					vector< DMatch > auxMatches;
					vector< DMatch > auxBestMatches;
					BFMatcher matcher(NORM_HAMMING,true);
					matcher.match(vecDesc[i],vecDesc[i+1+j], auxBestMatches);
					sort(auxBestMatches.begin(),auxBestMatches.end(),sortByDist);
					auxBestMatches.erase(auxBestMatches.begin()+30,auxBestMatches.end());
					
					for(int k = 0 ; k < auxBestMatches.size();k++){
						Point2f pt1 = vecKp[i][auxBestMatches[k].queryIdx].pt;
						Point2f pt2 = vecKp[i+1+j][auxBestMatches[k].trainIdx].pt;
						errorInlier += 	projectPointError(pt1,pt2,hAux);
					}
				}

				hAux = homoNoMultiplicated[i].clone();
				for(int j = -1; j > -(beforeMatches+1) ; j--){
					vector< DMatch > auxMatches;
					vector< DMatch > auxBestMatches;
					BFMatcher matcher(NORM_HAMMING,true);
					matcher.match(vecDesc[i+j],vecDesc[i], auxBestMatches);
					sort(auxBestMatches.begin(),auxBestMatches.end(),sortByDist);
					auxBestMatches.erase(auxBestMatches.begin()+10,auxBestMatches.end());	
					
					for(int k = 0 ; k < auxBestMatches.size();k++){
						Point2f pt1 = vecKp[i+j][auxBestMatches[k].queryIdx].pt;
						Point2f pt2 = vecKp[i][auxBestMatches[k].trainIdx].pt;
						errorInlier += 	projectPointError(pt1,pt2,hAux);
					}
					hAux = homoNoMultiplicated[i+j] * hAux;
				}

				errorTotal += errorInlier;
				cout << "el error total es: " << errorInlier <<endl;
			}
		}

		/*

		*/
		Mat stitchImgs(){
			cout << "\033[1;32m Generando orthomosaico: ... ("<< strImgs.size()-1<< ")\033[0m"<< endl;
			for (int i = 1; i < imgs.size(); i++){
				cout.flush();
				boundBox = stitchWarp(boundBox, imgs[i], H[i])[0];
				// string res = "Imagenes/resultados/Pegado/resultados" + to_string(i) + ".png";
				// imwrite(res, boundBox);
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
