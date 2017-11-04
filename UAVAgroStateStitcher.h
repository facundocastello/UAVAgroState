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
		bool bound = false;
		double yMin=0;
		double yMax=0;
		double xMin=0;
		double xMax=0;
		int pondSize;
		int imgHeight;
		int imgWidth;
		int minKeypoints = 5000;
		
		UAVAgroStateStitcher(int tamano = 4,
					float kPoints = 3,
					bool originalsize=false,
					int pondSize=1
					)
		{
			this->tamano = tamano;
			this->kPoints = kPoints;
			this->originalsize=originalsize;
			this->pondSize = pondSize;
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
						scene.at<Vec4b>(i,j) = obj.at<Vec4b>(i,j);
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


		Mat compareMats(Mat obj, Mat scene, Mat homoMatrix){
			Mat objAux,tmp;
			warpPerspective(obj, objAux, homoMatrix, Size(scene.cols, scene.rows));
			threshold(objAux, tmp, 1, 255, THRESH_BINARY);
			objAux = abs((scene & objAux) - objAux);
			// CommonFunctions::showWindowNormal(objAux);
							
			return objAux;
		}
		/*
		usando paralelismo obtengo todos los keypoints y descriptores
		*/
		void detectAndDescript(vector<Mat> imgs,
			 vector<Mat> &vecDesc,
			 vector< vector<KeyPoint> > &vecKp){
			///CALCULA LOS KEYPOINTS Y DESCRIPTORES DE CADA IMAGEN
			//-- Step 1 and 2 : Detect the keypoints and Calculate descriptors 
			vecDesc = vector<Mat>(imgs.size());
			vecKp = vector< vector<KeyPoint> >(imgs.size());
			cout << vecKp.size() <<endl;


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
							erode(mask, mask, kernel, Point(1, 1), 4);
						}
							
						orb->detectAndCompute(imgs[i] , mask , keypoints , descriptors);

						cout << "\n KeyPoint imagen" + to_string(i) + ": " << keypoints.size()<< endl;
						if(keypoints.size() < minKeypoints){
							cout << "recalculando keypoints " + to_string(i) <<endl;
						}
						kTres--;
					}
					vecDesc[i]=descriptors;
					vecKp[i]=keypoints;
				}
			});
		}
		/*usada en caso de que se quieran obtener los descriptores de un archivo .yml*/
		Mat readDetectAndComputeDesc(int i){
			struct timeval begin;
			gettimeofday(&begin, NULL);
			FileStorage fs("Data/DetectCompute/descriptores.yml", FileStorage::READ);
			Mat descriptor;
			fs["descriptor"+ to_string(i)] >> descriptor;
			begin = CommonFunctions::tiempo(begin, "Tiempo para cargar desc: ");

			return descriptor;
		}
		/*usada en caso de que se quieran obtener los keypoints de un archivo .yml*/
		vector<Point2f> readDetectAndComputeKp(int i){
			struct timeval begin;
			gettimeofday(&begin, NULL);
			FileStorage fs("Data/DetectCompute/descriptores.yml", FileStorage::READ);
			FileNode features = fs["features"+ to_string(i)];
			// iterate through a sequence using FileNodeIterator
			vector<Point2f> Kp;
			for(FileNodeIterator it = features.begin() ; it != features.end(); ++it )
			{
				Point2f pt((float)(*it)["x"], (float)(*it)["y"]);
				Kp.push_back(pt);
			}
			begin = CommonFunctions::tiempo(begin, "Tiempo para cargar kp: ");
			return Kp;
		}
		/*Realiza matchs entre los keypoints de 2 imagenes, en base a sus
		descriptores, y esto es acelerado usando paralelismo*/
		vector< vector< DMatch > > matchKp(vector<Mat> vecDesc){
			vector< vector< DMatch > > vecMatch(vecDesc.size());
			BFMatcher matcher(NORM_HAMMING,true);
			parallel_for_(Range(0, vecDesc.size()-1), [&](const Range& range){
				for(int i = range.start;i < range.end ; i++){
					cout << "Empezo Match "+to_string(i)+ ". \n";
					vector< DMatch > matches;
					matcher.match(vecDesc[i],vecDesc[i+1], matches);
					vecMatch[i] = matches;
					cout << "Termino Match "+to_string(i)+ " con "+ to_string(vecMatch[i].size()) + " matches.\n";	
				}
			});
			return vecMatch;
		}
		/*
		elimino matches 'erroneos' usando como criterio para saber si son malos
		o buenos el hecho de que entre cada imagen hay un desplazamiento
		*/
		vector< DMatch > goodMatches(vector< DMatch > match,vector< vector<KeyPoint> > keypoints,	double porcMinY,double porcMaxY,int minvalx, int maxvalx){
			vector< DMatch > gm;
			bool imgMismaAltura = false;
			int minvaly = imgHeight*porcMinY;
			int maxvaly = imgHeight*porcMaxY;
			while(gm.size() < 4){
				gm= vector< DMatch >();
				for(int i = 0; i < match.size(); i++){
					Point2f pt0 = keypoints[0][match[i].queryIdx].pt;
					Point2f pt1 = keypoints[1][match[i].trainIdx].pt;
					if(this->bound){
						pt0.y-=abs(this->yMin);
						pt0.x-=abs(xMin);
					}
					if( (pt1.y-pt0.y) > minvaly && (pt1.y-pt0.y) < maxvaly 
						&& abs(pt1.x-pt0.x) >= minvalx && abs(pt1.x-pt0.x) < maxvalx
						){
						gm.push_back(match[i]);
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
				sort(match.begin(),match.end(),sortByDist);
				gm.insert(gm.end(),match.begin(),match.begin()+50);
			}
			
			return gm;
		}
		/*
		obtengo varias homografias modificando ciertos parametros, y elijo la que
		sea mas adecuada
		*/
		vector<Mat> getHomography(Mat img1, Mat img2, int numMatch,vector< vector<KeyPoint> > keypoints, Mat lastH, vector< DMatch > match )
		{
			Mat H;
			double minHomoX = 9999;
			double minHomoY=9999;
			double porcMinY = 0.05;
			double porcMaxY = 0.5;
			int minvalx = 0;
			int maxvalx = 50;
			int bestvalx;double bestPorcMinY;double bestPorcMaxY;
			int vueltasI=10;int vueltasJ=10;int vueltasK=20;
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
							maxvalx = 50-k;
							good_matches[k] = goodMatches(match,keypoints,porcMinY,porcMaxY,minvalx,maxvalx);
							// sort(good_matches[k].begin(),good_matches[k].end(),sortByDist);
							///OBTENGO LOS PUNTOS EN LOS QUE SE ENCUENTRAN LOS GOOD MATCHES
							std::vector<Point2f> obj;
							std::vector<Point2f> scene;
							for (int l = 0; l < good_matches[k].size(); l++) {
								obj.push_back(keypoints[0][good_matches[k][l].queryIdx].pt);
								scene.push_back(keypoints[1][good_matches[k][l].trainIdx].pt);
							}
							// ARMO LA MATRIZ DE HOMOGRAFIA EN BASE A LOS PUNTOS ANTERIORES
							Mat maskH;
							if(good_matches[k].size() > 4){
								auxH[k] = findHomography(scene, obj, CV_RANSAC,3,mask_inliers[k]);
							}
						}});
					for(int k=0;k<vueltasK;k++){
						if(!auxH[k].empty()){
							Mat prodH=lastH*auxH[k];
							// compareMats(img1,img2,prodH);
							// por trigonometria aplico lo siguiente
							double auxHomoX = abs( .9*pow(prodH.at<double>(0,0),2) + .1*pow(prodH.at<double>(0,1),2) - 1 );
							double auxHomoY = abs( .1*pow(prodH.at<double>(1,0),2) + .9*pow(prodH.at<double>(1,1),2) - 1 );
							if(auxHomoX < minHomoX && auxHomoY < minHomoY){
								H = auxH[k];
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
			Mat aux;
			drawMatches(img1, keypoints[0],img2,keypoints[1],best_matches,aux,
			Scalar::all(-1),Scalar::all(-1),
			std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(numMatch)+".png",aux);
			vector< DMatch > best_inliers;
			for(int i = 0 ; i < best_matches.size();i++){
				if(MaskInliers.at<uchar>(0,i)){
					best_inliers.push_back(best_matches[i]);
				}
			}
			drawMatches(img1, keypoints[0],img2,keypoints[1],best_inliers,aux,
				Scalar::all(-1),Scalar::all(-1),
				std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(numMatch)+"inliers.png",aux);

			return{ H };
		}
		/*
		obtengo varias homografias modificando ciertos parametros, y elijo la que
		sea mas adecuada
		*/
		vector<Mat> getHomographyBigImages(Mat img1, Mat img2, int numMatch,vector< vector<KeyPoint> > keypoints, Mat lastH, vector< DMatch > match )
		{
			Mat H;
			vector< DMatch > good_matches;
			sort(match.begin(),match.end(),sortByDist);
			// good_matches = goodMatches(match,keypoints,.0,.2,0,1000);
			good_matches.insert(good_matches.end(),match.begin(),match.begin()+30);

			///OBTENGO LOS PUNTOS EN LOS QUE SE ENCUENTRAN LOS GOOD MATCHES
			std::vector<Point2f> obj;
			std::vector<Point2f> scene;
			for (int l = 0; l < good_matches.size(); l++) {
				obj.push_back(keypoints[0][good_matches[l].queryIdx].pt);
				scene.push_back(keypoints[1][good_matches[l].trainIdx].pt);
			}
			// ARMO LA MATRIZ DE HOMOGRAFIA EN BASE A LOS PUNTOS ANTERIORES
			H = findHomography(scene, obj, CV_RANSAC);
			Mat aux;
			if(img1.channels() == 4){
				vector <Mat> bgra;
				split(img1,bgra);
				Mat bgr[3] = {bgra[0],bgra[1],bgra[2]};
				merge(bgr,3,img1);
			}
			if(img2.channels() == 4){
				vector <Mat> bgra;
				split(img2,bgra);
				Mat bgr[3] = {bgra[0],bgra[1],bgra[2]};
				merge(bgr,3,img2);
			}
			drawMatches(img1, keypoints[0],img2,keypoints[1],good_matches,aux,
			Scalar::all(-1),Scalar::all(-1),
			std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imwrite("Imagenes/resultados/Pegado/matches/matchs"+to_string(numMatch)+".png",aux);
			return{ H };
		}
		/*
		Utilizo todas las funciones anteriores para realizar el stitching,
		siguiento el siguiente proceso:
		obtengo keypoints y descriptores - los matcheo - obtengo homografias
		- genero boundbox - pego las imagenes
		*/
		Mat stitchImgs(vector<string> strImgs){
			string normal = "\033[0m";
			string process = "\033[1;32m";
			vector<Mat> H(strImgs.size());
			vector<Mat> homoNoMultiplicated(strImgs.size());
			struct timeval begin;
			gettimeofday(&begin, NULL);
			H[0] = (Mat::eye(3, 3, CV_64F));
			homoNoMultiplicated[0] = (Mat::eye(3, 3, CV_64F));

			vector<Mat> imgs = CommonFunctions::cargarImagenes(strImgs , tamano);
			imgHeight =imgs[0].rows;
			imgWidth =imgs[0].cols;
			begin = CommonFunctions::tiempo(begin, "cargar las imagenes:");
			vector<Mat> vecDesc;
			vector< vector<KeyPoint> > vecKp;
			
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Obteniendo keypoints y descriptores: " + normal<< endl;
			detectAndDescript(imgs, vecDesc, vecKp);
			begin = CommonFunctions::tiempo(begin, "obtener keypoints:");
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Matcheando: " + normal << endl;
			//usado para documentar las homografias
			vector< vector< DMatch > > vecMatch = matchKp(vecDesc);
			begin = CommonFunctions::tiempo(begin, "realizar matching:");
			FileStorage fsHomo("Data/Homografias/homografias.yml", FileStorage::WRITE);
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Homografias: " + normal << endl;
			for(int i = 0; i < strImgs.size()-1;i++){
				cout << "Realizando Homografia "+to_string(i)+ ". \n";
				//Una homografia es para calcular el boundbox (H) y la otra es para
				//con ese boundbox calcular las otras homografias, multiplicandolas 
				//esta es homonomultpilicates
				vector<Mat> aux;
				if(imgs[i].channels()==4){
					aux = getHomographyBigImages(imgs[i],imgs[i+1],i+1,{vecKp[i],vecKp[i+1]},H[i],vecMatch[i]);
				}else{
					aux = getHomography(imgs[i],imgs[i+1],i+1,{vecKp[i],vecKp[i+1]},H[i],vecMatch[i]);
				}

				homoNoMultiplicated[i+1] = (aux[0]);
				H[i+1] = (H[i] * homoNoMultiplicated[i+1]);
				H[i+1] = H[i+1] / H[i+1].at<double>(2,2);
				//Encuentro el maximo y minimo tanto en x como en y de todas las 
				//homografias para despues poder hacer el bounding box;
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

			Mat boundBox = imgs[0];
			yMin -= 500;
			cout<< "ymin: "<< yMin << " ymax: "<< yMax<< "xmin: "<< xMin << " xmax: "<< xMax << endl;

			if(abs(yMin) > (imgHeight * imgs.size()/2)	|| 	abs(yMax) > (imgHeight * imgs.size()/2)
				|| 	abs(xMin) > (imgWidth * imgs.size()/2)	|| 	abs(xMax) > (imgWidth * imgs.size()/2)){
			cout<< " mal pegado "<<(abs(yMin) > (imgHeight * imgs.size()/2) )
			<<(abs(yMax) > (imgHeight* imgs.size()/2))<<(abs(xMin) > (imgWidth * imgs.size()/2))
			<<(abs(xMax) > (imgWidth * imgs.size()/2))<<endl;
			return Mat();
			}
			/*genero una imagen en negro con el tamaño que va a tener el ortomosaico
			y pego la imagen respetando los bordes calculados con las homografias */
			boundBox = CommonFunctions::boundingBox(boundBox, abs(yMin) , yMax , abs(xMin),xMax);	
			//adapto los keypoints de la primer imagen, al boundbox generado con esta
			Point2f ptAux(abs(xMin),abs(yMin));
			for(int i=0;i<vecKp[0].size();i++){
				vecKp[0][i].pt+=ptAux;
			}
			//vuelvo a calcular la homografia para la imagen con bordes
			bound=true;
			vector<Mat> aux;
			if(imgs[1].channels()==4){
				aux = getHomographyBigImages(boundBox,imgs[1],40,{vecKp[0],vecKp[1]},H[0],vecMatch[0]);
			}else{
				aux = getHomography(boundBox,imgs[1],40,{vecKp[0],vecKp[1]},H[0],vecMatch[0]);
			}
			
			/*tengo que adaptar todas las homografias a las nuevas dimensiones
			definidas por el boundbox*/
			begin = CommonFunctions::tiempo(begin, " obtener las homografias: ");
			H[1] = aux[0];
			for(int i = 2 ; i < H.size(); i++){
				H[i] = H[i-1] * homoNoMultiplicated[i];
				H[i] = H[i] / H[i].at<double>(2,2);
			}
			//adapto los parametros necesarios para que al pegar, se pegue la imagen grande
			if(originalsize){	
				FileStorage fsHomo("Data/Homografias/homografias2.yml", FileStorage::WRITE);

				Mat project_down = (Mat::eye(3, 3, CV_64F));
				project_down.at<double>(0,0)/=tamano;
				project_down.at<double>(1,1)/=tamano;
				project_down.at<double>(0,2)/=(tamano*2);
				project_down.at<double>(1,2)/=(tamano*2);
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
				boundBox = imgs[0];
				boundBox = CommonFunctions::boundingBox(boundBox, abs(yMin) , yMax , abs(xMin),xMax);	
				fsHomo.release();
			}
		

			//USANDO LAS HOMOGRAFIAS, COMIENZO EL PEGADO DE LAS IMAGENES
			cout << process + "-|-|-|-|-|-|-|-|-|-|-|-|-|Generando orthomosaico: ... ("<< strImgs.size()-1<< ")" + normal<< endl;
			for (int i = 1; i < strImgs.size(); i++){
				cout << "-" << (i+1) * 100 / strImgs.size() << "%" << endl;
				boundBox = stitchWarp(boundBox, imgs[i], H[i])[0];
				string res = "Imagenes/resultados/Pegado/resultados" + to_string(i) + ".png";
				imwrite(res, boundBox);
			}

			begin = CommonFunctions::tiempo(begin, " pegar imagenes: ");
			if(boundBox.channels() < 4){
				boundBox = CommonFunctions::addTransparence(boundBox);
			}

			return boundBox;
		}
		
};

#endif
