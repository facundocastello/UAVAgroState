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
				int rowGrid = 10;
				int colGrid = 3;
				float rowOverlap = 0.7;
				float colOverlap = 0.7;
				int position = 0;
				//position: -abDer = 0 -abIzq = 1 -arDer = 2 -arIzq = 3
				int tamano = 4;
				bool undistort = false;
				int kPoints = 1	;
				
				if(argc > 2){
					if( atoi(argv[2]) == 0 ){
						cout<<"The order of the params is:	\n   tamano \n   undistorted \n";
						cout<<"   position (abDer = 0 -abIzq = 1 -arDer = 2 -arIzq = 3) \n   kPoints \n   rowGrid \n   colGrid \n";
						return 0;
					}
					tamano = stoi(argv[2]);
				}
				if(argc > 3){
					undistort = stoi(argv[3]);
				}
				if(argc > 4){
					position = stoi(argv[4]);
				}
				if(argc > 5){
					kPoints = stoi(argv[5]);
				}
				if(argc > 6){
					rowGrid = stoi(argv[6]);
				}
				if(argc > 7){
					colGrid = stoi(argv[7]);
				}
				vector<string> strImgs;
				if(undistort){
					strImgs = CommonFunctions::obtenerImagenes("Imagenes/Undistort/");
				}else{
					strImgs = CommonFunctions::obtenerImagenes("Imagenes/Pegado/");
				}
			
				UAVAgroStateStitcher *uav = new UAVAgroStateStitcher(
					tamano,position,rowGrid,colGrid,
					rowOverlap,colOverlap,kPoints);
				Mat img = uav->stitchImgs(strImgs);
						
				imwrite("Imagenes/resultados/Pegado/resultado.png",img);
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