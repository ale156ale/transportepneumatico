/*
 * Desenvolvimento de um circuito eletrônico para leituras digitais da pressão estática,
 * temperatura e rotação do ventilador com um microcontrolador arduino para uma unidade   
 * piloto de transporte pneumático
 * Autor: Rodney Gomes da Silva
 * Data: Jul/2021
*/
#include <dht.h>        // Biblioteca do sensor de temperatura e umidade
#include "Nextion.h"    // Biblioteca da tela Nextion
#include <HX711.h>      // Bilioteca do conversor HX711 para o sensores de pressão
#include <EEPROM.h>     // Biblioteca para da memória EEPROM do arduino

#define dht_dpin A0     // Define que o sensor DHT está ligado na entrada A0

dht DHT;                // Declara a variável dht como uma instância da classe DHT

int pino_D0 = 2;        // Declara que o sensor de rotação está ligado na entrada 2
int rpm;                // Declara a variável rpm para armazenar a rotação calculada
float velocidade;         // Variável para armazenar a velocidade do ar em m/s
int TEMPO_GRAVAR=0;     // Variável para definir o tempo de gravação dos fatores na memória
//volatile byte pulsos;
int pulsos;
unsigned long timeold;

int EM_CALIBRACAO = 0;

const int BOTAO_MAIS = 50;  // Fator +
const int BOTAO_MENOS = 51; // Fator -
const int BOTAO_ZERO = 52;  // Zero

int ESTADO_MAIS = 0;
int ESTADO_MENOS = 0;
int ESTADO_ZERO = 0;

double S1;              // Variável que armazena o valor medido no sensor S1
double S2;              // Variável que armazena o valor medido no sensor S2
double S3;              // Variável que armazena o valor medido no sensor S3
double S4;              // Variável que armazena o valor medido no sensor S4
double S5;              // Variável que armazena o valor medido no sensor S5
double S6;              // Variável que armazena o valor medido no sensor S6
double S7;              // Variável que armazena o valor medido no sensor S7
double S8;              // Variável que armazena o valor medido no sensor S8
double S9;              // Variável que armazena o valor medido no sensor S9
double S10;             // Variável que armazena o valor medido no sensor S10
double S11;             // Variável que armazena o valor medido no sensor S11
double S12;             // Variável que armazena o valor medido no sensor S12

//////////////////////////////////////////////////////////////
//double FATOR_S1 = 6830;                                     // 
//double FATOR_S2 = 6469;                                     //    
//double FATOR_S3 = 6790;                                     //
//double FATOR_S4 = 6770;                                     //
//double FATOR_S5 = 5665;                                     //
//double FATOR_S6 = 5844;                                     ///////////////////////////
//double FATOR_S7 = 6580;                                     // Fatores de Calibração //
//double FATOR_S8 = 6120;                                     ///////////////////////////
//double FATOR_S9 = 6810;                                     //
//double FATOR_S10 = 6815;                                    //
//double FATOR_S11 = 6805;                                    //
//double FATOR_S12 = 6785;                                    //
//unsigned FATOR_ROT = 10;                                    //
//////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
double FATOR_S1;                                     // 
double FATOR_S2;                                     //    
double FATOR_S3;                                     //
double FATOR_S4;                                     //
double FATOR_S5;                                     //
double FATOR_S6;                                     ///////////////////////////
double FATOR_S7;                                     // Fatores de Calibração //
double FATOR_S8;                                     ///////////////////////////
double FATOR_S9;                                     //
double FATOR_S10;                                    //
double FATOR_S11;                                    //
double FATOR_S12;                                    //
unsigned FATOR_ROT;                                    //
////////////////////////////////////////////////////////////

//uint32_t FATOR_S1;
//uint32_t FATOR_S2;
//uint32_t FATOR_S3;
//uint32_t FATOR_S4;
//uint32_t FATOR_S5;
//uint32_t FATOR_S6;
//uint32_t FATOR_S7;
//uint32_t FATOR_S8;
//uint32_t FATOR_S9;
//uint32_t FATOR_S10;
//uint32_t FATOR_S11;
//uint32_t FATOR_S12;
//uint32_t FATOR_ROT;

uint32_t CAL_S1;
uint32_t CAL_S2;
uint32_t CAL_S3;
uint32_t CAL_S4;
uint32_t CAL_S5;
uint32_t CAL_S6;
uint32_t CAL_S7;
uint32_t CAL_S8;
uint32_t CAL_S9;
uint32_t CAL_S10;
uint32_t CAL_S11;
uint32_t CAL_S12;
uint32_t CAL_ROT;

int SPAN=4000;             // Valor máximo de pressão em mmca nos sensores

void contador()
{
  //Incrementa contador
  pulsos++;
}

//////////////////////////////////////////////////////////////
  HX711 scale1(39,41);                                      //
  HX711 scale2(45,43);                                      //
  HX711 scale3(47,49);                                      //
  HX711 scale4(40,38);                                      //
  HX711 scale5(42,44);                                      //////////////////////////////////
  HX711 scale6(48,46);                                      // Define os pinos onde estão   //
  HX711 scale7(35,37);                                      // ligados os conversores HX711 //
  HX711 scale8(33,31);                                      //////////////////////////////////
  HX711 scale9(27,29);                                      //
  HX711 scale10(36,34);                                     //
  HX711 scale11(30,32);                                     //
  HX711 scale12(28,26);                                     //
//////////////////////////////////////////////////////////////

int err, vel;
float temp, humi;

NexNumber txt_Vel  = NexNumber(1, 1, "txt_Vel");
NexNumber txt_Temp = NexNumber(1, 18, "Temp");
NexNumber txt_ROT  = NexNumber(1, 3, "txt_ROT");
NexNumber txt_Umid = NexNumber(1, 4, "txt_Umid");
NexNumber txt_S1 = NexNumber(1, 5, "txt_S1");
NexNumber txt_S2 = NexNumber(1, 6, "txt_S2");
NexNumber txt_S3 = NexNumber(1, 7, "txt_S3");
NexNumber txt_S4 = NexNumber(1, 8, "txt_S4");
NexNumber txt_S5 = NexNumber(1, 9, "txt_S5");
NexNumber txt_S6 = NexNumber(1, 10, "txt_S6");
NexNumber txt_S7 = NexNumber(1, 11, "txt_S7");
NexNumber txt_S8 = NexNumber(1, 12, "txt_S8");
NexNumber txt_S9 = NexNumber(1, 13, "txt_S9");
NexNumber txt_S10 = NexNumber(1, 14, "txt_S10");
NexNumber txt_S11 = NexNumber(1, 15, "txt_S11");
NexNumber txt_S12 = NexNumber(1, 16, "txt_S12");

NexNumber txt_S1_c = NexNumber(2, 3, "txt_S1_c");
NexNumber txt_S2_c = NexNumber(3, 3, "txt_S2_c");
NexNumber txt_S3_c = NexNumber(4, 3, "txt_S3_c");
NexNumber txt_S4_c = NexNumber(5, 3, "txt_S4_c");
NexNumber txt_S5_c = NexNumber(6, 3, "txt_S5_c");
NexNumber txt_S6_c = NexNumber(7, 3, "txt_S6_c");
NexNumber txt_S7_c = NexNumber(8, 3, "txt_S7_c");
NexNumber txt_S8_c = NexNumber(9, 3, "txt_S8_c");
NexNumber txt_S9_c = NexNumber(10, 3, "txt_S9_c");
NexNumber txt_S10_c = NexNumber(11, 3, "txt_S10_c");
NexNumber txt_S11_c = NexNumber(12, 3, "txt_S11_c");
NexNumber txt_S12_c = NexNumber(13, 3, "txt_S12_c");
NexNumber txt_ROT_c = NexNumber(14, 3, "txt_ROT_c");

NexNumber txt_Fator_S1 = NexNumber(2, 4, "txt_Fator_S1");
NexNumber txt_Fator_S2 = NexNumber(3, 4, "txt_Fator_S2");
NexNumber txt_Fator_S3 = NexNumber(4, 4, "txt_Fator_S3");
NexNumber txt_Fator_S4 = NexNumber(5, 4, "txt_Fator_S4");
NexNumber txt_Fator_S5 = NexNumber(6, 4, "txt_Fator_S5");
NexNumber txt_Fator_S6 = NexNumber(7, 4, "txt_Fator_S6");
NexNumber txt_Fator_S7 = NexNumber(8, 4, "txt_Fator_S7");
NexNumber txt_Fator_S8 = NexNumber(9, 4, "txt_Fator_S8");
NexNumber txt_Fator_S9 = NexNumber(10, 4, "txt_Fator_S9");
NexNumber txt_Fator_S10 = NexNumber(11, 4, "txt_Fator_S10");
NexNumber txt_Fator_S11 = NexNumber(12, 4, "txt_Fator_S11");
NexNumber txt_Fator_S12 = NexNumber(13, 4, "txt_Fator_S12");
NexNumber txt_Fator_ROT = NexNumber(14, 4, "txt_Fator_ROT");

NexNumber CALIBRAR_S1 = NexNumber(2, 5, "CALIBRAR_S1");
NexNumber CALIBRAR_S2 = NexNumber(3, 5, "CALIBRAR_S2");
NexNumber CALIBRAR_S3 = NexNumber(4, 5, "CALIBRAR_S3");
NexNumber CALIBRAR_S4 = NexNumber(5, 5, "CALIBRAR_S4");
NexNumber CALIBRAR_S5 = NexNumber(6, 5, "CALIBRAR_S5");
NexNumber CALIBRAR_S6 = NexNumber(7, 5, "CALIBRAR_S6");
NexNumber CALIBRAR_S7 = NexNumber(8, 5, "CALIBRAR_S7");
NexNumber CALIBRAR_S8 = NexNumber(9, 5, "CALIBRAR_S8");
NexNumber CALIBRAR_S9 = NexNumber(10, 5, "CALIBRAR_S9");
NexNumber CALIBRAR_S10 = NexNumber(11, 5, "CALIBRAR_S10");
NexNumber CALIBRAR_S11 = NexNumber(12, 5, "CALIBRAR_S11");
NexNumber CALIBRAR_S12 = NexNumber(13, 5, "CALIBRAR_S12");
NexNumber CALIBRAR_ROT = NexNumber(14, 5, "CALIBRAR_ROT");

NexTouch *nex_listen_list[] =
{
  &CALIBRAR_S1,
  &CALIBRAR_S2,  
  &CALIBRAR_S3,
  &CALIBRAR_S4,
  &CALIBRAR_S5,
  &CALIBRAR_S6,
  &CALIBRAR_S7,
  &CALIBRAR_S8,
  &CALIBRAR_S9,
  &CALIBRAR_S10,
  &CALIBRAR_S11,
  &CALIBRAR_S12,
  &CALIBRAR_ROT,
  NULL
};

void LER_FATORES_MEMORIA()
{
  FATOR_S1 = EEPROM.get(0, FATOR_S1);
  FATOR_S2 = EEPROM.get(10, FATOR_S2);
  FATOR_S3 = EEPROM.get(20, FATOR_S3);
  FATOR_S4 = EEPROM.get(30, FATOR_S4);
  FATOR_S5 = EEPROM.get(40, FATOR_S5);
  FATOR_S6 = EEPROM.get(50, FATOR_S6);
  FATOR_S7 = EEPROM.get(60, FATOR_S7);
  FATOR_S8 = EEPROM.get(70, FATOR_S8);
  FATOR_S9 = EEPROM.get(80, FATOR_S9);
  FATOR_S10 = EEPROM.get(90, FATOR_S10);
  FATOR_S11 = EEPROM.get(100, FATOR_S11);
  FATOR_S12 = EEPROM.get(110, FATOR_S12);
  FATOR_ROT = EEPROM.get(120, FATOR_ROT);
}

void GRAVAR_FATORES_PADRAO()
{
// Valores antigos, calibrado em bancada
//  FATOR_S1 = 6830;//  FATOR_S2 = 6469;//  FATOR_S3 = 6790;//  FATOR_S4 = 6770;//  FATOR_S5 = 5665;//  FATOR_S6 = 5844;
//  FATOR_S7 = 6580;//  FATOR_S8 = 6120;//  FATOR_S9 = 6810;//  FATOR_S10 = 6815;//  FATOR_S11 = 6805;//  FATOR_S12 = 6785;
//  unsigned FATOR_ROT = 10;

//Valores novos, calibrados no dia 09/11/2019 no laboratório da Unisanta
  FATOR_S1 = 14460;
  FATOR_S2 = 13908;
  FATOR_S3 = 15020;
  FATOR_S4 = 14551;
  FATOR_S5 = 12241;
  FATOR_S6 = 12894;
  FATOR_S7 = 14440;
  FATOR_S8 = 14399;
  FATOR_S9 = 15721;
  FATOR_S10 = 16293;
  FATOR_S11 = 14612;
  FATOR_S12 = 14820;
  unsigned FATOR_ROT = 1;

  
  EEPROM.put(0,FATOR_S1); 
  EEPROM.put(10,FATOR_S2);
  EEPROM.put(20,FATOR_S3);
  EEPROM.put(30,FATOR_S4);
  EEPROM.put(40,FATOR_S5);
  EEPROM.put(50,FATOR_S6);
  EEPROM.put(60,FATOR_S7);
  EEPROM.put(70,FATOR_S8);
  EEPROM.put(80,FATOR_S9);
  EEPROM.put(90,FATOR_S10);
  EEPROM.put(100,FATOR_S11);
  EEPROM.put(110,FATOR_S12);
  EEPROM.put(120,FATOR_ROT);  

}

void ESCREVER_FATORES_MEMORIA()
{
  EEPROM.put(0,FATOR_S1); 
  EEPROM.put(10,FATOR_S2);
  EEPROM.put(20,FATOR_S3);
  EEPROM.put(30,FATOR_S4);
  EEPROM.put(40,FATOR_S5);
  EEPROM.put(50,FATOR_S6);
  EEPROM.put(60,FATOR_S7);
  EEPROM.put(70,FATOR_S8);
  EEPROM.put(80,FATOR_S9);
  EEPROM.put(90,FATOR_S10);
  EEPROM.put(100,FATOR_S11);
  EEPROM.put(110,FATOR_S12);
  EEPROM.put(120,FATOR_ROT);  
}

void ESCREVER_FATORES_TELA()
{
  txt_Fator_S1.setValue(FATOR_S1);
  txt_Fator_S2.setValue(FATOR_S2);
  txt_Fator_S3.setValue(FATOR_S3);
  txt_Fator_S4.setValue(FATOR_S4);
  txt_Fator_S5.setValue(FATOR_S5);
  txt_Fator_S6.setValue(FATOR_S6);
  txt_Fator_S7.setValue(FATOR_S7);
  txt_Fator_S8.setValue(FATOR_S8);
  txt_Fator_S9.setValue(FATOR_S9);
  txt_Fator_S10.setValue(FATOR_S10);
  txt_Fator_S11.setValue(FATOR_S11);
  txt_Fator_S12.setValue(FATOR_S12);
  txt_Fator_ROT.setValue(FATOR_ROT);  
}

void LER_CALIBRAR()
{
  CALIBRAR_S1.getValue(&CAL_S1);
  CALIBRAR_S2.getValue(&CAL_S2);
  CALIBRAR_S3.getValue(&CAL_S3);
  CALIBRAR_S4.getValue(&CAL_S4);
  CALIBRAR_S5.getValue(&CAL_S5);
  CALIBRAR_S6.getValue(&CAL_S6);
  CALIBRAR_S7.getValue(&CAL_S7);
  CALIBRAR_S8.getValue(&CAL_S8);
  CALIBRAR_S9.getValue(&CAL_S9);
  CALIBRAR_S10.getValue(&CAL_S10);
  CALIBRAR_S11.getValue(&CAL_S11);
  CALIBRAR_S12.getValue(&CAL_S12);
  CALIBRAR_ROT.getValue(&CAL_ROT);

  if (CAL_S1 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale1.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S1 = FATOR_S1 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S1 = FATOR_S1 - 100;}        
    txt_Fator_S1.setValue(FATOR_S1);
    scale1.set_scale(FATOR_S1); 
    txt_S1_c.setValue(S1);
  }

  if (CAL_S2 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale2.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S2 = FATOR_S2 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S2 = FATOR_S2 - 100;}            
    txt_Fator_S2.setValue(FATOR_S2);
    scale2.set_scale(FATOR_S2); 
    txt_S2_c.setValue(S2);
  }

  if (CAL_S3 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale3.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S3 = FATOR_S3 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S3 = FATOR_S3 - 100;}            
    txt_Fator_S3.setValue(FATOR_S3);
    scale3.set_scale(FATOR_S3); 
    txt_S3_c.setValue(S3);
  }

  if (CAL_S4 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale4.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S4 = FATOR_S4 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S4 = FATOR_S4 - 100;}            
    txt_Fator_S4.setValue(FATOR_S4);
    scale4.set_scale(FATOR_S4); 
    txt_S4_c.setValue(S4);
  }

  if (CAL_S5 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale5.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S5 = FATOR_S5 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S5 = FATOR_S5 - 100;}            
    txt_Fator_S5.setValue(FATOR_S5);
    scale5.set_scale(FATOR_S5); 
    txt_S5_c.setValue(S5);
  }

  if (CAL_S6 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale6.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S6 = FATOR_S6 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S6 = FATOR_S6 - 100;}            
    txt_Fator_S6.setValue(FATOR_S6);
    scale6.set_scale(FATOR_S6); 
    txt_S6_c.setValue(S6);
  }  

  if (CAL_S7 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale7.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S7 = FATOR_S7 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S7 = FATOR_S7 - 100;}            
    txt_Fator_S7.setValue(FATOR_S7);
    scale7.set_scale(FATOR_S7); 
    txt_S7_c.setValue(S7);
  }  

  if (CAL_S8 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale8.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S8 = FATOR_S8 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S8 = FATOR_S8 - 100;}            
    txt_Fator_S8.setValue(FATOR_S8);
    scale8.set_scale(FATOR_S8); 
    txt_S8_c.setValue(S8);
  }  

  if (CAL_S9 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale9.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S9 = FATOR_S9 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S9 = FATOR_S9 - 100;}            
    txt_Fator_S9.setValue(FATOR_S9);
    scale9.set_scale(FATOR_S9); 
    txt_S9_c.setValue(S9);
  }  

  if (CAL_S10 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale10.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S10 = FATOR_S10 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S10 = FATOR_S10 - 100;}            
    txt_Fator_S10.setValue(FATOR_S10);
    scale10.set_scale(FATOR_S10); 
    txt_S10_c.setValue(S10);
  }    

  if (CAL_S11 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale11.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S11 = FATOR_S11 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S11 = FATOR_S11 - 100;}            
    txt_Fator_S11.setValue(FATOR_S11);
    scale11.set_scale(FATOR_S11); 
    txt_S11_c.setValue(S11);
  }  

  if (CAL_S12 == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale12.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_S12 = FATOR_S12 + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_S12 = FATOR_S12 - 100;}            
    txt_Fator_S12.setValue(FATOR_S12);
    scale12.set_scale(FATOR_S12); 
    txt_S12_c.setValue(S12);
  }  

  if (CAL_ROT == 1){
    if (digitalRead(BOTAO_ZERO) == HIGH){scale12.tare();}      
    if (digitalRead(BOTAO_MAIS) == HIGH){FATOR_ROT = FATOR_ROT + 100;}
    if (digitalRead(BOTAO_MENOS) == HIGH){FATOR_ROT = FATOR_ROT - 100;}            
    txt_Fator_ROT.setValue(FATOR_ROT);
    scale12.set_scale(FATOR_ROT); 
    txt_ROT_c.setValue(rpm);
  }          
}

void CALIBRAR_SENSORES()
{
  scale1.set_scale(FATOR_S1); 
  scale2.set_scale(FATOR_S2); 
  scale3.set_scale(FATOR_S3); 
  scale4.set_scale(FATOR_S4);
  scale5.set_scale(FATOR_S5);
  scale6.set_scale(FATOR_S6);
  scale7.set_scale(FATOR_S7);
  scale8.set_scale(FATOR_S8);
  scale9.set_scale(FATOR_S9);
  scale10.set_scale(FATOR_S10);
  scale11.set_scale(FATOR_S11);
  scale12.set_scale(FATOR_S12);
}

void LER_VALOR_SENSORES()
{
  S1 = 2 * scale1.get_units(1);
  S2 = 2 * scale2.get_units(1);
  S3 = 2 * scale3.get_units(1);
  S4 = 2 * scale4.get_units(1);
  S5 = 2 * scale5.get_units(1);
  S6 = 2 * scale6.get_units(1);
  S7 = 2 * scale7.get_units(1);
  S8 = 2 * scale8.get_units(1);
  S9 = 2 * scale9.get_units(1);
  S10 = 2 * scale10.get_units(1);
  S11 = 2 * scale11.get_units(1);
  S12 = 2 * scale12.get_units(1);

  if (S1<0){ S1=0;}
  if (S2<0){ S2=0;}
  if (S3<0){ S3=0;}
  if (S4<0){ S4=0;}
  if (S5<0){ S5=0;}
  if (S6<0){ S6=0;}
  if (S7<0){ S7=0;}
  if (S8<0){ S8=0;}
  if (S9<0){ S9=0;}
  if (S10<0){ S10=0;}
  if (S11<0){ S11=0;}
  if (S12<0){ S12=0;}

  if (S1>SPAN){ S1=SPAN;}
  if (S2>SPAN){ S2=SPAN;}
  if (S3>SPAN){ S3=SPAN;}
  if (S4>SPAN){ S4=SPAN;}
  if (S5>SPAN){ S5=SPAN;}
  if (S6>SPAN){ S6=SPAN;}
  if (S7>SPAN){ S7=SPAN;}
  if (S8>SPAN){ S8=SPAN;}
  if (S9>SPAN){ S9=SPAN;}
  if (S10>SPAN){ S10=SPAN;}
  if (S11>SPAN){ S11=SPAN;}
  if (S12>SPAN){ S12=SPAN;}
 
}

void ESCREVER_SENSORES_TELA()
{
  txt_Vel.setValue(velocidade);
  txt_Temp.setValue(temp);
  txt_ROT.setValue(rpm);
  txt_Umid.setValue(humi);
  txt_S1.setValue(S1);
  txt_S2.setValue(S2); 
  txt_S3.setValue(S3);
  txt_S4.setValue(S4);
  txt_S5.setValue(S5);
  txt_S6.setValue(S6); 
  txt_S7.setValue(S7);
  txt_S8.setValue(S8);
  txt_S9.setValue(S9);
  txt_S10.setValue(S10); 
  txt_S11.setValue(S11);
  txt_S12.setValue(S12);  
}

void CALCULA_VELOCIDADE()
{
  //velocidade=(2.6438*(S11-S12)) - 5.9977; Fórmula de 27/08
  //velocidade=(0.9679*(S11-S12)); Fórmula de 17/10
  //velocidade=(0.5340*(S11-S12)); //Fórmula de 30/11
    //velocidade=(0.0299*pow(S11-S12,3))-(1.2898*pow(S11-S12,2))+(19.432*(S11-S12))-(88.333); Testado 30/11 sem sucesso
  //velocidade=(4.1557*(S11-S12)) - 17.844; //Fórmula de 08/08

  //velocidade=(5.1972*(S11-S12)) - 7.5036;  

velocidade=((8.3156+((S11/2)-(S12/2)))/9.2002) * 10;

  if (S11<=0){ velocidade=0;}  
  if (S12<=0){ velocidade=0;}  
  
  if (velocidade<0){ velocidade=0;}  
  
}

void CALCULA_ROTACAO()
{
  //Atualiza contador a cada segundo
  if (millis() - timeold >= 1000)
  {
    //Desabilita interrupcao durante o calculo
//    detachInterrupt(0);
    //rpm = (60 * 100 / 1 ) / (millis() - timeold) * pulsos;
    rpm=((pulsos*pulsos/3)/100)*2.5;
    timeold = millis();
    pulsos = 0;
//    attachInterrupt(0, contador, FALLING);  
  }
}

// Ponto  RPM     ARD
// 3      5,8     11
// 4      8,9     13
// 5      11,5    17
// 6      14,4    16
// 7      17,5    18
// 8      20,4    20
// 9      22,8    23
// 10     25,0    25

//void CALCULA_ROTACAO()
//{
//  pulsos=0;
//  attachInterrupt(0, contador, FALLING);
//  delay(1000);
//  detachInterrupt(0);
//  pulsos=pulsos*60/16;
//  rpm=pulsos;
//     
//  
//}

void ZERAR_TODOS(){
  scale1.tare();  
  scale2.tare();  
  scale3.tare();  
  scale4.tare();  
  scale5.tare();  
  scale6.tare();  
  scale7.tare();  
  scale8.tare();  
  scale9.tare();  
  scale10.tare();  
  scale11.tare();  
  scale12.tare();    
}

void setup()
{

  pinMode(BOTAO_MAIS,INPUT);
  pinMode(BOTAO_MENOS,INPUT);
  pinMode(BOTAO_ZERO,INPUT);
  
  Serial.begin(115200);
  nexInit();
  nexLoop(nex_listen_list);

  //GRAVAR_FATORES_PADRAO();
  LER_FATORES_MEMORIA();

  CALIBRAR_SENSORES();
    
  ZERAR_TODOS();
  
  pinMode(pino_D0, INPUT);
  attachInterrupt(0, contador, FALLING);
  pulsos = 0;
  rpm = 0;
  timeold = 0;

  ESCREVER_FATORES_TELA();
     
}

void loop()
{
  if (digitalRead(BOTAO_ZERO) == HIGH){ZERAR_TODOS();}      
  nexLoop(nex_listen_list);
  DHT.read11(dht_dpin);
  humi = DHT.humidity;
  temp = DHT.temperature;
  LER_VALOR_SENSORES();  
  CALCULA_VELOCIDADE();
  CALCULA_ROTACAO();
  ESCREVER_SENSORES_TELA();
  LER_CALIBRAR();
  //EM_CALIBRACAO = CAL_S1 + CAL_S2 + CAL_S3 + CAL_S4 + CAL_S5 + CAL_S6 + CAL_S7 + CAL_S8 + CAL_S9 + CAL_S10 + CAL_S11 + CAL_S12 + CAL_ROT;
  //if (EM_CALIBRACAO == 0){ESCREVER_SENSORES_TELA();}

//  TEMPO_GRAVAR++;
//  if (TEMPO_GRAVAR>=15){          // Espera 1 minuto para gravar os fatores de calibração na memória do arduino
//  ESCREVER_FATORES_MEMORIA();   // O número de gravações é limitado a 100 000 gravações
//    TEMPO_GRAVAR=0;
//  }

