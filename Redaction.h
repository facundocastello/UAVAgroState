#ifndef Redaction_H
#define Redaction_H

#include "hpdf.h"
#include <setjmp.h>
#include "IndexCalculation.h"

/**
 * @brief Redacta el PDF final.
 * 
 */
class Redaction{
    public:
		/**
		 * @brief Crea un PDF con todas las imágenes.
		 * 
		 * @return int 
		 */
        int generatePDF(){
			//Se usa un booleano para verificar si se quieren sobre-escribir los informes
			cout << "Desea sobre-escribir los informes ya redactados?(0 no, 1 si)";
			bool sobreescribir;
			cin >> sobreescribir;
			cout << CommonFunctions::stringAzul("Comenzando proceso de redacción: ")+"\n"; 
			//se inicializa el contador de tiempo
			struct timeval beginAll;
			struct timeval begin;
			gettimeofday(&beginAll, NULL);
			gettimeofday(&begin, NULL);
			//se inicializa el pdf
            HPDF_Doc pdf;
			HPDF_Error_Handler error_handler;
			jmp_buf env;
			pdf = HPDF_New (error_handler, NULL);
			if (!pdf) {
				printf ("ERROR: cannot create pdf object.\n");
				return 1;
			}
			if (setjmp(env)) {
				HPDF_Free (pdf);
				return 1;
			}
			//se obtienen las ubicaciones de las imágenes y carpetas
			IndexCalculation ic;
			vector<string> strImgsName = ic.obtenerMSOutput();
			vector<string> strIndexs = CommonFunctions::obtenerImagenes((strImgsName[0]+"/").c_str());
			vector<string> strImgs = CommonFunctions::obtenerImagenes((strIndexs[0]+"/").c_str());
			//se recorren las carpetas dentro de cada "ortomosaico"
			for(int i = 0 ; i < strImgsName.size() ; i++){
				string strImgName = CommonFunctions::obtenerUltimoDirectorio2(strImgsName[i]);
				string ubicacion = "Data/Informes/" + strImgName+".pdf";
				if(sobreescribir || !CommonFunctions::existFile(ubicacion)){
					cout << CommonFunctions::stringAzul("Redactando información de la imágen: " + strImgName ) + "\n"; 
					//se obtienen los indices
					vector<string> strIndexs = CommonFunctions::obtenerImagenes((strImgsName[i]+"/").c_str());
					// se crea un pdf nuevo para cada ortomosaico
					pdf = HPDF_New (error_handler, NULL);
					HPDF_SetCompressionMode (pdf, HPDF_COMP_ALL);
					//se escribe la imagen original
					writeOriginal(strImgName,pdf);
					for(int j = 0 ; j < strIndexs.size() ; j++){
						string strIndex = CommonFunctions::obtenerUltimoDirectorio2(strIndexs[j]);
						cout << CommonFunctions::stringAzul(" - Redactando información del indice: " + strIndex ) + "\n"; 
						//se escriben las imagenes y textos correspondientes al indice, lut y chart.
						writeIndex(strImgName,strIndex, pdf);
						writeLut(strImgName, strIndex, pdf);
						writeChart(strImgName, strIndex, pdf);
					}
					//se escribe el pdf
					cout << CommonFunctions::stringAzul(" - Escribiendo PDF " + ubicacion ) + "\n"; 
					CommonFunctions::escribirPDF(pdf,ubicacion);
					begin = CommonFunctions::tiempo(begin, "Terminar " + strImgName +" :");
				}
			}
			CommonFunctions::tiempo(beginAll, "Terminar redacción:");
			// HPDF_Page_SetSize (page_1, HPDF_PAGE_SIZE_B5, HPDF_PAGE_LANDSCAPE);
            return 0;
        }
		/**
		 * @brief Se escribe el ortomosaico y un texto que lo acompaña.
		 * 
		 * @param strImgName 
		 * @param pdf 
		 */
		void writeOriginal(string strImgName, HPDF_Doc pdf){
			uav::Stitcher stitch;
			string strOriginalImg = stitch.obtenerOutputRF() + strImgName + ".png";
			HPDF_Page page_1;
			page_1 = HPDF_AddPage (pdf);
			
			writeText("Imagen: " + strImgName,pdf,page_1,25,50,800,550,100,HPDF_TALIGN_CENTER);
			// string text = generateChartText(strImgName, strIndex);
			// writeText(text,pdf,page_1,12,50,700,550,100);

			// page_1 = HPDF_AddPage (pdf);
			
			writeImg(pdf, page_1, strOriginalImg.c_str(),450,650,50,50);
		}
		/**
		 * @brief Se escribe el indice y un texto que lo acompaña.
		 * 
		 * @param strImgName 
		 * @param strIndex 
		 * @param pdf 
		 */
		void writeIndex(string strImgName,string strIndex, HPDF_Doc pdf){
			IndexCalculation ic;			
			string strIndexImg = ic.obtenerIndex(strImgName, strIndex);
			HPDF_Page page_1;
			page_1 = HPDF_AddPage (pdf);
			
			writeText("Indice: " + strIndex,pdf,page_1,25,50,800,550,100,HPDF_TALIGN_CENTER);
			// string text = generateChartText(strImgName, strIndex);
			// writeText(text,pdf,page_1,12,50,700,550,100);

			// page_1 = HPDF_AddPage (pdf);
			
			writeImg(pdf, page_1, strIndexImg.c_str(),450,650,50,50);
		}
		/**
		 * @brief Se escribe el mapa de colores y un texto que lo acompaña.
		 * 
		 * @param strImgName 
		 * @param strIndex 
		 * @param pdf 
		 */
		void writeLut(string strImgName,string strIndex, HPDF_Doc pdf){
			IndexCalculation ic;			
			string strLutImg = ic.obtenerLut(strImgName, strIndex);
			HPDF_Page page_1;
			page_1 = HPDF_AddPage (pdf);
			
			writeText("Mapa de colores: " + strIndex,pdf,page_1,25,50,800,550,100,HPDF_TALIGN_CENTER);
			string text = "Para facilitar la visualización del indice "+strIndex+" se aplica un mapa de colores";
			writeText(text,pdf,page_1,12,50,750,550,100);

			// page_1 = HPDF_AddPage (pdf);
			
			writeImg(pdf, page_1, strLutImg.c_str(),450,625,50,25);
		}
		/**
		 * @brief Se escribe el gráfico de porcentajes y un texto que lo acompaña.
		 * 
		 * @param strImgName 
		 * @param strIndex 
		 * @param pdf 
		 */
		void writeChart(string strImgName,string strIndex, HPDF_Doc pdf){
			IndexCalculation ic;			
			string strChart = ic.obtenerChart(strImgName, strIndex);
			string strChartImg = ic.obtenerChartImg(strImgName, strIndex);
			HPDF_Page page_1;
			page_1 = HPDF_AddPage (pdf);
			
			writeText("Cuantización y gráfico: " + strIndex,pdf,page_1,25,50,800,550,100,HPDF_TALIGN_CENTER);
			string text = generateChartText(strImgName, strIndex);
			writeText(text,pdf,page_1,12,50,750,550,100);

			page_1 = HPDF_AddPage (pdf);
			
			writeImg(pdf, page_1, strChartImg.c_str(),450,670,50,160);
			writeImg(pdf, page_1, strChart.c_str(),300,100,280,50);
		}
		/**
		 * @brief Genera el texto para el gráfico, utilizando metadatos manejador por FSManager.
		 * 
		 * @param strImgName 
		 * @param strIndex 
		 * @return string 
		 */
		string generateChartText(string strImgName, string strIndex){
			FSManager fs(strImgName + ".yml", "imagen");
			string text="Para facilitar la distinsión entre la vegetacion saludable, la no saludable y la tierra, se realizó una cuantización utilizando diferentes colores y asignandolos a multiples intervalos de valores. Con esto se obtuvieron los siguientes valores: \n";
			vector<float> porcentajes = fs.readVFloat("porcentaje"+strIndex);
			vector<int> limites = fs.readVInt("limites"+strIndex);
			float hectareas = fs.readFloat("hectareas");
			for(int i=0; i < porcentajes.size(); i++){
				if(porcentajes[i] > 0.1){
					text+= "        *" + to_string(porcentajes[i]*hectareas/100) + " hectareas tienen valores entre " + to_string(limites[i]) + " y " + to_string(limites[i+1]) + "\n";
				}
			}
			text+= "Además se aplicó la cuantización al ortomosaico y se genero un gráfico que muestra en que porcentaje participa cada cuantizado.";
			return text;
		}
		/**
		 * @brief Escribe un texto en el rectangulo indicado por las coordenadas que se pasan por parametros.
		 * 
		 * @param text 
		 * @param pdf 
		 * @param page_1 
		 * @param sizeFont 
		 * @param left 
		 * @param top 
		 * @param right 
		 * @param bottom 
		 * @param textAlign 
		 */
		void writeText(string text, HPDF_Doc pdf, HPDF_Page page_1,int sizeFont,int left,int top,int right,int bottom, HPDF_TextAlignment textAlign = HPDF_TALIGN_JUSTIFY){
			/* Print the title of the page (with positioning center). */
			int height = HPDF_Page_GetHeight (page_1);
    		int width = HPDF_Page_GetWidth (page_1);
			HPDF_Page_BeginText (page_1);
			
			// HPDF_UseUTFEncodings(pdf);
			// HPDF_SetCurrentEncoder(pdf, "UTF-8");
			HPDF_Font  font = HPDF_GetFont(pdf, HPDF_LoadTTFontFromFile(pdf, "arial.ttf", HPDF_TRUE), NULL);
			HPDF_Page_SetFontAndSize(page_1, font, sizeFont);
			HPDF_Rect rect;
			/* HPDF_TALIGN_LEFT */
			rect.left = left;
			rect.top = top;
			rect.right = right;
			rect.bottom = bottom;
			HPDF_STATUS ret = HPDF_Page_TextRect (page_1, rect.left, rect.top, rect.right, rect.bottom,
			text.c_str(), textAlign, NULL);
			HPDF_Page_EndText (page_1);
		}
		/**
		 * @brief Escribe una imágen dentro de un pdf, con su relación ancho/largo mantenida pero redimensionada.
		 * 
		 * @param pdf 
		 * @param page 
		 * @param file 
		 * @param maxWidth 
		 * @param maxHeight 
		 * @param xSpace 
		 * @param ySpace 
		 * @return int 
		 */
        int writeImg(HPDF_Doc pdf,HPDF_Page page,const char* file,int maxWidth = 600,int maxHeight = 840,int xSpace = 50,int ySpace = 50){
            try {
				HPDF_Image img = HPDF_LoadPngImageFromFile(pdf,file);
				int width = HPDF_Image_GetWidth(img);
				int height = HPDF_Image_GetHeight(img);
				float resolucion;
				if(width > maxWidth){
					resolucion = (float)height/(float)width;
					width = maxWidth;
					height = width*resolucion;
				}
				if(height > maxHeight){
					resolucion = (float)width/(float)height;
					height = maxHeight;
					width = height*resolucion;
				}
				// cout << resolucion << " " << width << " " << height; 
                xSpace += (maxWidth - width)/2;
                ySpace += (maxHeight - height)/2;
				HPDF_Page_DrawImage(page,img,xSpace,ySpace,width,height);		
			} catch (...) {
				HPDF_Free (pdf);
				return 1;
			}

            return 0;
        }


};

#endif