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
	int altura;
	int page = 0;
	int x0 = 80;
	int x1 = 520;
	int lineSpacing = 17;
	string fonts = "cmunbsr.ttf";
	vector<int> sizeImg;
		/**
		 * @brief Crea un PDF con todas las imágenes.
		 * 
		 * @return int 
		 */
        int generatePDF(){
			//Se usa un booleano para verificar si se quieren sobre-escribir los informes
			cout << "Desea sobre-escribir los informes ya redactados?(0 no, 1 si)";
			altura = 740;
			sizeImg.push_back(500);
			sizeImg.push_back(500);
			sizeImg.push_back(50);
			sizeImg.push_back(100);
			
			bool sobreescribir = true;
			// cin >> sobreescribir;
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
					writePortada(pdf,strImgName);
					writeOriginal(strImgName,pdf);
					for(int j = 0 ; j < strIndexs.size() ; j++){
						string strIndex = CommonFunctions::obtenerUltimoDirectorio2(strIndexs[j]);
						cout << CommonFunctions::stringAzul(" - Redactando información del indice: " + strIndex ) + "\n"; 
						//se escriben las imagenes y textos correspondientes al indice, lut y chart.
						writeIndex(strImgName,strIndex, pdf);
						writeCuantizado(strImgName,strIndex, pdf);
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
		void writePortada(HPDF_Doc pdf,string strName){
			IndexCalculation ic;	
			HPDF_Page page_1;
			page_1 = HPDF_AddPage (pdf);
			HPDF_Page_SetTextLeading(page_1,lineSpacing);
			
			fonts = "cmunsx.ttf";

			writeText("Informe periodico del cultivo",pdf,page_1,25,x0-10,350,x1+10,100,HPDF_TALIGN_CENTER);
			writeText("Fecha de generacion del informe: " + CommonFunctions::nowDate(),pdf,page_1,20,x0-10,250,x1+10,100,HPDF_TALIGN_CENTER);

			fonts = "cmunbsr.ttf";
			
			
			writeImg2(pdf, page_1, "UAVAgroState.png",300,300,160,400);
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
			HPDF_Page_SetTextLeading(page_1,lineSpacing);
			FSManager fs(strImgName,"imagen");

			writeContext("Ortomosaico " + strImgName, " ", strImgName,pdf, page_1);

			writeText(" - El orto-mosaico fue redimensionada a la escala 1/" + to_string(fs.readInt("tamano")),pdf,page_1,12,x0+10,altura-40,x1-10,100,HPDF_TALIGN_LEFT);
			writeText(" - Las imagenes fueron capturadas a una altura de " + to_string(fs.readInt("altura")) + "m",pdf,page_1,12,x0+10,altura-60,x1-10,100,HPDF_TALIGN_LEFT);
			writeText(" - La dimension que representa el ortomosaico son " + CommonFunctions::fToS(fs.readFloat("hectareas")) + "ha",pdf,page_1,12,x0+10,altura-80,x1-10,100,HPDF_TALIGN_LEFT);
			writeText(" - Fecha de captura de las imagenes: "+CommonFunctions::nameToDate(strImgName) ,pdf,page_1,12,x0+10,altura-100,x1-10,100,HPDF_TALIGN_LEFT);
			
			writeImg(pdf, page_1, strOriginalImg.c_str(),sizeImg[0],sizeImg[1],sizeImg[2],sizeImg[3]);
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
			HPDF_Page_SetTextLeading(page_1,lineSpacing);
			
			writeContext("Indice de vegetación " + strIndex, strIndex, strImgName,pdf, page_1);

			string text = "El índice de vegetación es una combinación de las bandas espectrales para producir un simple valor que indique la cantidad o vigor de vegetación dentro de un píxel. Permitiéndonos estimar y evaluar el estado de salud de la vegetación, en base a la medición de la radiación que las plantas emiten o reflejan. Se representa en escala de grises donde a mayor intensidad, mas saludable es la vegeteción.";
			writeText(text,pdf,page_1,12,x0+10,altura-40,x1-10,100);
			
			writeImg(pdf, page_1, strIndexImg.c_str(),sizeImg[0],sizeImg[1],sizeImg[2],sizeImg[3]);
		}

		void writeCuantizado(string strImgName,string strIndex, HPDF_Doc pdf){
			IndexCalculation ic;			
			string strIndexImg = ic.obtenerCuant(strImgName, strIndex);
			HPDF_Page page_1;
			page_1 = HPDF_AddPage (pdf);
			HPDF_Page_SetTextLeading(page_1,lineSpacing);
			
			writeContext("Cuantificación", strIndex, strImgName,pdf, page_1);

			string text = "    Se implentó una técnica de compresión con pérdida que consiste en comprimir un rango de valores a un único valor y, de esta forma, cuando el número de símbolos discretos en un flujo dado se reduce, el flujo se vuelve más comprensible. A este procedimiento se le denomina cuantificación del color.";
			writeText(text,pdf,page_1,12,x0+10,altura-40,x1-10,100);
			
			writeImg(pdf, page_1, strIndexImg.c_str(),sizeImg[0],sizeImg[1],sizeImg[2],sizeImg[3]);
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
			HPDF_Page_SetTextLeading(page_1,lineSpacing);
			
			writeContext("Mapa de colores", strIndex, strImgName,pdf, page_1);

			string text = "    La percepción humana no está construida para observar cambios finos en las imágenes en escala de grises, por lo que a menudo se necesita colorear las imágenes para obtener una mejor percepcion. Para lograr esto se aplica un mapa de colores que relaciona cada intensidad de la escala de grises con un color.";
			writeText(text,pdf,page_1,12,x0+10,altura-40,x1-10,100);

			// page_1 = HPDF_AddPage (pdf);
			
			writeImg(pdf, page_1, strLutImg.c_str(),sizeImg[0],sizeImg[1],sizeImg[2],sizeImg[3]);
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
			HPDF_Page_SetTextLeading(page_1,lineSpacing);
			
			writeContext("Cuantizacion y grafico", strIndex, strImgName, pdf, page_1);
			string text = generateChartText(strImgName, strIndex);
			writeText(text,pdf,page_1,12,x0+10,altura-40,x1-10,100);

			page_1 = HPDF_AddPage (pdf);
			HPDF_Page_SetTextLeading(page_1,lineSpacing);
			
			writeContext(" ", strIndex, strImgName, pdf, page_1,false);
			writeImg(pdf, page_1, strChartImg.c_str(),sizeImg[0],sizeImg[1],sizeImg[2],sizeImg[3]);
			writeImg(pdf, page_1, strChart.c_str(),300,altura,150,310);
		}
		/**
		 * @brief Genera el texto para el gráfico, utilizando metadatos manejador por FSManager.
		 * 
		 * @param strImgName 
		 * @param strIndex 
		 * @return string 
		 */
		string generateChartText(string strImgName, string strIndex){
			FSManager fs(strImgName, "imagen");
			string text="    Se busca representar los datos de una manera que se puedan realizar comparaciones con otros ortomosaicos, sean de una parcela diferente, de otro tipo de cultivo o de otro periodo de tiempo. Para facilitar la distinsion entre la vegetacion saludable, la no saludable y la tierra, se realizo una cuantizacion utilizando diferentes colores y asignandolos a multiples intervalos de valores. \n Con esto se obtuvieron los siguientes valores: \n";
			vector<float> porcentajes = fs.readVFloat("porcentaje"+strIndex);
			vector<int> limites = fs.readVInt("limites"+strIndex);
			float hectareas = fs.readFloat("hectareas");
			for(int i=0; i < porcentajes.size(); i++){
				if(porcentajes[i] > 0.1){
					text+= "        * " + to_string(porcentajes[i]*hectareas/100) + " hectareas tienen valores entre " + to_string(limites[i]) + " y " + to_string(limites[i+1]) + "\n";
				}
			}
			text+= "    Ademas se aplico la cuantizacion al ortomosaico y se genero un grafico que muestra en que porcentaje participa cada cuantizado.";
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

			HPDF_Font  font = HPDF_GetFont(pdf, HPDF_LoadTTFontFromFile(pdf, fonts.c_str(), HPDF_TRUE), NULL);
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

		int writeImg2(HPDF_Doc pdf,HPDF_Page page,const char* file,int maxWidth = 600,int maxHeight = 840,int xSpace = 50,int ySpace = 50){
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
				HPDF_Page_DrawImage(page,img,xSpace,ySpace,width,height);		
			} catch (...) {
				HPDF_Free (pdf);
				return 1;
			}

            return 0;
        }

		void writeContext(string strText,string strIndex,string strName, HPDF_Doc pdf,HPDF_Page page_1, bool writeTitle=true){
			HPDF_Page_SetLineWidth (page_1, 1.0);
			//encabezado
			writeText("Ortomosaico " + strName,pdf,page_1,10,x0,altura+40,x1,altura,HPDF_TALIGN_LEFT);
			if(strIndex != " ")
				writeText("Indice "+strIndex,pdf,page_1,10,x0,altura+40,x1,altura,HPDF_TALIGN_RIGHT);
			draw_line(page_1,x0,x1,altura+20,altura+20);
			//cuerpo
			if(writeTitle)
				writeText(strText,pdf,page_1,20,x0+20,altura,x1-20,x0+10,HPDF_TALIGN_CENTER);
			//pie de pagina
			draw_line(page_1,x0,x1,840-(altura+20),840-(altura+20));
			writeText("Creado con UAVAgroState",pdf,page_1,12,300,840-(altura+20)-5,500,840-(altura+20)-20,HPDF_TALIGN_RIGHT);
			writeText(CommonFunctions::nowDate(),pdf,page_1,12,300,840-(altura+20)-20,500,840-(altura+20)-20,HPDF_TALIGN_RIGHT);
			writeImg2(pdf,page_1,"UAVAgroState.png",50,50,100,840-(altura+20)-55);
			writeText(to_string(++page),pdf,page_1,12,x0+10,840-(altura+20)-5,500,840-(altura+20)-20,HPDF_TALIGN_CENTER);
			
		}

		void draw_line  (HPDF_Page    page,
            float        x0,
			float        x1,
			float        y0,
            float        y1)
		{
			HPDF_Page_MoveTo (page, x0, y0);
			HPDF_Page_LineTo (page, x1, y1);
			HPDF_Page_Stroke (page);
		}

};

#endif