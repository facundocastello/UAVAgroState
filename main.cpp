#include "UAVAgroStateStitcher.h"
#include "funcionesutiles.h"
#include "opencv2/core/core.hpp"
#include "CommonFunctions.h"
#include "UAVAgroStateIndexCalcs.h"
#include "UAVAgroStateCalibration.h"

using namespace cv;

void readme();

int main(int argc, char** argv)
{
	int option = 0;
	if( argc > 1){
		option = atoi(argv[1]);
	}else{
		cout<<"Que deseas hacer? \n 1: Calibrar camara \n 2: Calcular orthomosaico \n 3: Calcular ndvi \n 4: Salir" << endl;
		cin>>option;
	}

	while(option != 4)
	{
		switch(option)
		{
			case 1:{
				int optionCalibration=0;
				if(argc > 2){
					optionCalibration = atoi(argv[2]);
				}else{
				cout << "Desea obtener la matriz para calibrar (1), o realizar la undistorcion (2)?"<<endl;
				cin >> optionCalibration;
				}
				switch(optionCalibration){
					case 1:{
						cout << "Obteneniendp la matriz para calibrar"<<endl;
						// int numBoards = 0;
						int numCornersHor;
						int numCornersVer;

						printf("Enter number of corners along width: ");
						scanf("%d", &numCornersHor);
						
						printf("Enter number of corners along height: ");
						scanf("%d", &numCornersVer);
					
						cout<<"name of the camera: "<<endl;
						string cameraName;
						cin>>cameraName;

						UAVAgroStateCalibration::calibrateImg(numCornersHor,numCornersVer,cameraName);
					}break;
					case 2:{
						cout << "Realizando la undistorcion"<<endl;
						cout<<"name of the camera: "<<endl;
						string cameraName;
						cin>>cameraName;
						UAVAgroStateCalibration::undistortImgs(cameraName);
					}break;
					}	
			}
			break;
			case 2:{
				//clock_t begin = clock();
				////Cargo los string de imagenes
				int tamano = 4;
				bool undistort = false;
				float kPoints = 3	;

				struct timeval begin;
				gettimeofday(&begin, NULL);
				
				if(argc > 2){
					if( atoi(argv[2]) == 0 ){
						cout<<"The order of the params is:	\n   tamano \n   undistorted \n";
						cout<<"   position (abDer = 0 -abIzq = 1 -arDer = 2 -arIzq = 3) \n   kPoints \n   rowGrid \n   colGrid \n";
						return 0;
					}
					tamano = stoi(argv[2]);
				}else{
					cout<< "Resize de las imagenes 1/n, ingrese n: ";
					cin>>tamano;
				}
				if(argc > 3){
					undistort = stoi(argv[3]);
				}else{
					cout<< "Quiere cargar las imagenes desde undistort? (1): ";
					cin>>undistort;
				}
				if(argc > 4){
					kPoints = stof(argv[4]);
				}else{
					cout<< "Que threshold ingresara? (1): ";
					cin>>kPoints;
				}

				vector<string> strImgs;
				if(undistort){
					strImgs = CommonFunctions::obtenerImagenes("Imagenes/Undistort/");
				}else{
					strImgs = CommonFunctions::obtenerImagenes("Imagenes/Pegado/");
				}

				UAVAgroStateStitcher *uav = new UAVAgroStateStitcher(
					tamano,kPoints);
				Mat img = uav->stitchImgs(strImgs);		
				imwrite("Imagenes/resultados/Pegado/resultado.png",img);
				
				CommonFunctions::tiempo(begin, "realizar todo: ");

				// for(int i = 0; i < strImgs.size(); i++){
				// 	UAVAgroStateStitcher *uav = new UAVAgroStateStitcher(
				// 		tamano,kPoints);
				// 	vector<string> strImgsAux;
					
				// 	strImgsAux.insert(strImgsAux.end(),strImgs.begin(), strImgs.begin() + i);
				// 	strImgsAux.insert(strImgsAux.end(),strImgs.begin() + i + 1, strImgs.end());
				// 	Mat img = uav->stitchImgs(strImgsAux, i);
				// 	imwrite("Imagenes/resultados/Pegado/resultado.png",img);
				// }
			}
			break;
			case 3:{
				vector<string> strNDVI = CommonFunctions::obtenerImagenes("Imagenes/NDVI/");
				for(int i = 0 ; i<strNDVI.size();i++){
					UAVAgroStateIndexCalcs::ndviCalcu(strNDVI[i]);
				}
			}
			break;
		}
		cout<<"Proceso finalizado, ahora que deseas hacer? \n 1: Calibrar camara \n 2: Calcular orthomosaico \n 3: Calcular ndvi \n 4: Salir" << endl;
		cin>>option;
	}

	
	
}