#ifndef Stitcher_H
#define Stitcher_H

#include <stdio.h>
#include <iostream>
#include "CommonFunctions.h"
using namespace cv;
using namespace std;

namespace uav{

	bool sortByDist(DMatch a, DMatch b){
		return a.distance < b.distance;
	}

	class Stitcher{
		public:
			int tamano;
			bool originalSize=false;
			int maxMatch = 100;
			double yMin=0;
			double yMax=0;
			double xMin=0;
			double xMax=0;
			int imgHeight;
			int imgWidth;
			int minKeypoints;
			bool usarHomografia;
			bool finalResult;
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

			Stitcher(
						int tamano = 4,
						int minKeypoints=5000,
						bool originalSize=false,
						bool usarHomografia =false,
						bool finalResult = false
						)
			{
				this->tamano = tamano;
				this->originalSize=originalSize;
				this->usarHomografia = usarHomografia;
				this->minKeypoints = minKeypoints;
				this->finalResult = finalResult;
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
				for(int i = 0 ; i < 7; i ++){
					blur(imgMaskFrame,imgMaskFrame,Size(100,100));
					Mat newImg;
					imgMaskFrame.copyTo(newImg,bgra[3]);	
					imgMaskFrame = newImg;
				}
				

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
						cout << "\n KeyPoint imagen" + to_string(i) + ": " + to_string(keypoints.size());
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
				best_inliers = vector< vector< DMatch > >(imgs.size()-1);
				BFMatcher matcher(NORM_HAMMING,true);
				int vecKpSize = vecKp.size();
				parallel_for_(Range(0, vecDesc.size()-1), [&](const Range& range){
					for(int i = range.start;i < range.end ; i++){
						// cout << "Empezo Match "+to_string(i)+ ". \n";
						vector< DMatch > matches;
						matcher.match(vecDesc[i],vecDesc[i+1], vecMatch[0][i]);
						
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
								auxHomoX[i] = pow(prodH.at<double>(0,0),2) + pow(prodH.at<double>(0,1),2) -1;
								auxHomoY[i] = pow(prodH.at<double>(1,0),2) + pow(prodH.at<double>(1,1),2) -1;
						}else{
							auxError[i] = 9999;
							auxHomoY[i] = 9999;	
							auxHomoX[i] = 9999;	
						}
					}
				});


				for(int i=minMatches;i<vueltasI;i++){
					if( abs(auxHomoX[i] + auxHomoY[i]) < minError){
						bestvalx = i;
						minHomoX = auxHomoX[i];
						minHomoY = auxHomoY[i];
						minError = abs(auxHomoX[i] + auxHomoY[i]);
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
				// drawMatches(removeAlpha(imgs[numHomo]), vecKp[numHomo],removeAlpha(imgs[numHomo+1]),vecKp[numHomo+1],best_inliers[numHomo],aux,
				// Scalar::all(-1),Scalar::all(-1),
				// vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
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
				H.clear();
				H = vector<Mat>();
				homoNoMultiplicated = vector<Mat>(imgs.size());
				H.push_back(Mat::eye(3, 3, CV_64F));
				homoNoMultiplicated[0] = (Mat::eye(3, 3, CV_64F));
				
				for(int i = 0; i < imgs.size()-1;i++){
					cout << "Realizando Homografia "+to_string(i)+ ". \n";
					//caso comun
					getHomography(i);

					H.push_back(H[i] * homoNoMultiplicated[i+1]);
					H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
					if(H[i+1].at<double>(0,0) < 0 && !finalResult){
						H.pop_back();
						break;
					}
				}

			}
			/*
			En base a las homografias, se obtienen los valores que van a delimitar al bound box,
			y evalua que estos no sean de una homogragia mal calculada
			*/
			bool findBoundBoxLimits(){
				cout << "\033[1;32mObteniendo bordes boundbox: \033[0m"<< endl;

				yMin=0;	yMax=0;	xMin=0;	xMax=0;
				for(int i = 0; i < H.size()-1; i++){
					// CUANDO ALGUNOS PARAMETROS SON MENOS QUE 0 ES PQ ESTA INCLINADA DE MANERA Q FAVORECE A MIN X
					Mat imgAux = imgs[i+1];
					Mat imgResta = imgs[0];
					vector<double> newX(4);
					vector<double> newY(4);
					newX[0] = H[i+1].at<double>(0,2);
					newY[0] = H[i+1].at<double>(1,2);

					newX[1] = H[i+1].at<double>(0,2) + H[i+1].at<double>(0,0) * imgAux.cols;
					newY[1] = H[i+1].at<double>(1,2) + H[i+1].at<double>(1,0) * imgAux.cols;

					newX[2] = H[i+1].at<double>(0,2) + H[i+1].at<double>(0,0) * imgAux.cols + H[i+1].at<double>(0,1) * imgAux.rows;
					newY[2] = H[i+1].at<double>(1,2) + H[i+1].at<double>(1,0) * imgAux.cols + H[i+1].at<double>(1,1) * imgAux.rows;

					newX[3] = H[i+1].at<double>(0,2) + H[i+1].at<double>(0,1) * imgAux.rows;
					newY[3] = H[i+1].at<double>(1,2) + H[i+1].at<double>(1,1) * imgAux.rows;

					if( (*min_element(newX.begin(),newX.end()) ) < xMin){
						xMin = (*min_element(newX.begin(),newX.end()) );
					}
					if( ((*max_element(newX.begin(),newX.end()) ) - imgResta.cols) > xMax){
						xMax = ((*max_element(newX.begin(),newX.end()) ) - imgResta.cols);
					}
					if( (*min_element(newY.begin(),newY.end()) ) < yMin){
						yMin = (*min_element(newY.begin(),newY.end()) );
					}
					if( ((*max_element(newY.begin(),newY.end()) ) - imgResta.rows) > yMax){
						yMax = ((*max_element(newY.begin(),newY.end()) ) - imgResta.rows);
					}
				}
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
				if(originalSize){
					cout << "\033[1;32m Resize al tamaño original: \033[0m" << endl;

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
					}
					yMin*=tamano;yMax*=tamano;xMin*=tamano;xMax*=tamano;
					
					for(int i = 0 ; i < H.size(); i++){
						imgs[i] = CommonFunctions::cargarImagen(strImgs[i] , 1,IMREAD_UNCHANGED);
					}
					removeCorners();

					boundBox = imgs[0];
					boundBox = CommonFunctions::boundingBox(boundBox, abs(yMin) , yMax , abs(xMin),xMax);
				}
			}

			/*
				En base a las homografias realiza el pegado de las imagenes
			*/

			void eraseFromVectors(){
				imgs.erase(imgs.begin());
				strImgs.erase(strImgs.begin());
				vecKp.erase(vecKp.begin());
				if(vecMatch[0].size() > 0){
					vecMatch[0].erase(vecMatch[0].begin());
					best_inliers.erase(best_inliers.begin());
				}
			}

			Mat stitchImgs(){
				cout << "\033[1;32m Generando orthomosaico: ... ("<< H.size()-1<< ")\033[0m"<< endl;
				eraseFromVectors();
				for (int i = 1; i < H.size(); i++){
					cout.flush();
					boundBox = stitchWarp(boundBox, imgs[0], H[i])[0];
					eraseFromVectors();
					cout << "-" << (i+1) * 100 / H.size() << "%";
				}
				cout<<endl;
				if(boundBox.channels() < 4){
					boundBox = CommonFunctions::addTransparence(boundBox);
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
				if(imgs.empty()){
					return Mat();
				}
				// compensateBright();

				removeCorners();

				begin = CommonFunctions::tiempo(begin, "cargar las imagenes:");

				imgHeight =imgs[0].rows;
				imgWidth =imgs[0].cols;

				detectAndDescript();
				begin = CommonFunctions::tiempo(begin, "obtener keypoints:");

				matchKp();
				begin = CommonFunctions::tiempo(begin, "realizar matching:");
				int numRes = 0;
				while(imgs.size() > 0){
					getHomographies();
					begin = CommonFunctions::tiempo(begin, " obtener las homografias: ");

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
					// multiBandStitch();
					begin = CommonFunctions::tiempo(begin, " generar el orthomosaico: ");
					if(finalResult){
						escribirOutputRF();
					}else{
						escribirOutputOrto(numRes);
					}
					numRes++;
				}

				return boundBox;
			}
			/*
			utilizo esta funcion para sperar el proceso en el que se generan las filas y en el que se pegan
			*/
			void processManager(){
				cout << "Comenzando proceso de pegado con: -Tamaño: "<< tamano << " -Recuperar tamaño original: "<<originalSize<< " -Minimos kp: "<< minKeypoints << endl;

				Mat img;
				while(strImgs.size() == 0){
					strImgs = obtenerInput();
					if(strImgs.size() == 0){
						cout << "Para comenzar ingrese las imagenes dentro de Imagenes/Pegado/input/ y presione entrer" << endl;
						getchar();
					}
				}
				
				img = runAll();

				if(img.empty()){
					cout << "\033[1;31m" << "Los archivos dentro de input, no son imágenes" << "\033[0m" << '\n';
				}else{
					minKeypoints = 10000;
					tamano = originalSize? tamano:1;
					strImgs = obtenerInputOrto();
					finalResult = true;
					img = runAll();
					
				}

			}

			vector<string> obtenerInput(){
				return CommonFunctions::obtenerImagenes("Imagenes/Pegado/input/");
			}
			vector<string> obtenerInputOrto(){
				return CommonFunctions::obtenerImagenes("Imagenes/Pegado/output/ortomosaico/");
			}
			string obtenerOutputRF(){
				return "Imagenes/Pegado/output/resultadofinal.png";
			}
			string obtenerOutputOrto(int num){
				return "Imagenes/Pegado/output/ortomosaico/resultado"+to_string(num)+".png";
			}

			void escribirOutputRF(){
				CommonFunctions::escribirImagen(obtenerOutputRF(),boundBox);
			}

			void escribirOutputOrto(int numRes){
				CommonFunctions::escribirImagen(obtenerOutputOrto(numRes),boundBox);
			}


	};
}

#endif
