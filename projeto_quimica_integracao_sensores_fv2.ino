/*integração das leituras dos sensores:
  -RGB TCS3200(cor rgb)
  -LDR(fotoresistencia)
  -APDS9960(Cor RGB e movimento(Não é utilizado nesse caso))
  -TSL2561(intensidade luminosa)
*/

#include <MD_TCS230.h>//Biblioteca para o sensor rgb tcs3200
#include <FreqCount.h>//Biblioteca para o sensor rgb tcs3200
/*mede a frequência de um sinal contando o número de pulsos durante um tempo fixo.*/

#include <Wire.h>
/*Comunicação I2C: 
 * SDA - Serial Data(Dados)
 * SCL - Serial Clock("Relogio")
 */

//Bibliotecas para o sensor rgb  TSL2561:
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

//Biblioteca sensor apds
#include <SparkFun_APDS9960.h>

// Definição de pinos sensor para o sensor tcs3200
#define  S2_OUT  12
#define  S3_OUT  13
#define  OE_OUT   8    // LOW = ENABLED
//OUT => Pino 5~

String dados, uni, value, desc,rgbData;
char espaco;


SparkFun_APDS9960 apds = SparkFun_APDS9960();//Criando uma instância(objeto) do sensor APDS9960 
uint16_t ambient_light = 0;// Variável para armazenar a luz ambiente
uint16_t red_light = 0;     // Variável para armazenar a luz vermelha
uint16_t green_light = 0;   // Variável para armazenar a luz verde
uint16_t blue_light = 0;    // Variável para armazenar a luz 

MD_TCS230  CS(S2_OUT, S3_OUT, OE_OUT);//instância do sensor rgb TCS3200.Definindo Pinos

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);//instância do sensor TSL2561


/*---------------------------------------------------------------------------------------------*/

void setup(){
   Serial.begin(9600);

   Serial.println(F("SEJA BEM VINDO!"));
   Serial.println(F("Sensores Inicializados!"));
   Serial.println(F("ATENÇÃO: UNIDADES DE MEDIDAS DISPONIVEIS PARA OS SENSORES: 'APDS', 'TSL', 'RGB' e 'LDR'."));
   Serial.println(F("Exemplo, siga esse modelo: '50 RGB'"));
   Serial.println(F("----------------------------------------------------------------------------------------\n"));
}


void loop(){
  if(Serial.available() > 0){
    dados = Serial.readStringUntil('\n');//Lendo inputs da porta serial
    //Serial.println(dados);

    espaco = dados.indexOf(' ');//quebrando a string até o espaço ' '
    //Serial.println(espaco);

    value = dados.substring(0, espaco);//valor numerico da string
     // Serial.println(value);
      
    uni = dados.substring(espaco+1);//valor da unidade da string(+1 carrega na variavel a parti do caractere " "(espaço)+1 da string, até o final
      
    uni.toUpperCase();

    String dataSensor;
    
    //INDENTIFICANDO FUNÇÃO:
/*--Lendo Sensor LDR--------------------------------------------------------------*/
    if(uni == "ldr" || uni == "LDR"){
      int ldr = readSensorLDR();

      Serial.println(F("Lendo LDR..."));
      //Loop para validar se ldr é diferente de 0 antes de imprimir
      int cont;
      
      while(ldr == 0){
        ldr = readSensorLDR();

        if(cont >= 40){
          break;
        }
        delay(100);
        cont++;
      }

      dataSensor = String(ldr);

/*--Lendo Sensor RGB--------------------------------------------------------------*/
    }else if(uni == "rgb" || uni == "RGB"){
      String dataRGB = readSensorRGB();

      Serial.println(F("Aguarde. Lendo Sensor RGB..."));
      //loop para compensar o atraso de leitura e só imprimir quando ele retornar um valor 
      int cont;
      
      while(dataRGB == ""){
        //Serial.println("Buscando dados");
        dataRGB = readSensorRGB();
        //Serial.println(dataRGB);

        if(cont >= 40){
          break;
        }
        delay(100);
        cont++;
      }

      dataSensor = dataRGB;
      
/*--Lendo Sensor TSL--------------------------------------------------------------*/
    }else if(uni == "tsl" || uni == "TSL"){
      String lux = readSensorTSL();

      Serial.println("Lendo TSL...");
      //if para verificar se não houve erro e só imprimir se estiver tudo certo
      int cont;
      
      while(lux != ""){
         lux = readSensorTSL();

        if(cont >= 40){
          break;
        }
        delay(100);
        cont++;
      }

      dataSensor = lux;
/*--Lendo Sensor APDS--------------------------------------------------------------*/
    }else if(uni == "apds" || uni == "APDS"){
      String dataAPDS = readSensorAPDS();
      int cont;

      while(dataAPDS == ""){
        dataAPDS = readSensorAPDS();

        if(cont >= 40){
          break;
        }
        delay(100);
        cont++;
      }

      dataSensor = dataAPDS;
/*--INPUT INVALIDO OU NÃO INDENTIFICADO-----------------------------------------------*/     
    }else{
      Serial.println("Input invalido");
    }
/*--TERMINO DE LEITURAS--------------------------------------------------------------*/
    Serial.println(F("_______________________________________________________________________________________"));
    if(dataSensor == ""){
      Serial.println(F("Algo deu errado! Tente novamente"));
    }else{
      Serial.println(uni + ":" + "\t" + value + "\t"+dataSensor);
    }
    Serial.println(F("_______________________________________________________________________________________\n"));
    
    delay(1000);
  }  
}

int readSensorLDR(){
  int ldr = analogRead(A0);

  return ldr;
}

String readSensorRGB(){
 //Config:
 CS.begin();//inicializando sensor

 static  bool  waiting;

 String rgbString;
 
  if (!waiting)
  {
    CS.read();
    waiting = true;
  }
  else
  {
    if (CS.available())
    {
      colorData  rgb;
      
      CS.getRGB(&rgb);

      int red = rgb.value[TCS230_RGB_R];
      int green = rgb.value[TCS230_RGB_G];
      int blue = rgb.value[TCS230_RGB_B];
  
      rgbString = "RGB [" + String(red) + "," + String(green) + "," + String(blue) + "]";
      //Serial.println(rgbString);
      
      waiting = false;
    }
  }
  return rgbString;
}

String readSensorTSL(){
  //Config:
  if(configSensorTSL() == true){
      sensors_event_t event;//variavel do tipo sensors_event_t    
      /*sensors_event_t: estrutura de dados que é usada para armazenar os resultados da leitura do sensor.*/
      
      tsl.getEvent(&event);//lendo dados do sensor e atualizando na variavel event

      String lux;
      /* Exibir os resultados (a luz é medida em lux) */
      if (event.light){
        lux = String(event.light) +  " lux";
        //Serial.print(event.light); Serial.println(" lux");
        
        disableSensorTSL();
        return lux;
      }
      else{
        /* Se event.light = 0 lux, o sensor está provavelmente saturado
           e nenhum dado confiável pôde ser gerado! */
        Serial.println(F("Sobrecarga do sensor"));
      }
    
  }else{
    Serial.println(F("Erro na inicialização do sensor TSL2561!"));
  }
  return "";
  disableSensorTSL();
}

String readSensorAPDS(){
  //Config:
  String dataAPDS;
  
  if(configSensorAPDS() == true){
          if (  !apds.readAmbientLight(ambient_light) ||
          !apds.readRedLight(red_light) ||
          !apds.readGreenLight(green_light) ||
          !apds.readBlueLight(blue_light) ) {
      Serial.println("Erro ao ler os valores de luz");
    } else {
      dataAPDS = "Ambiente: " + String(ambient_light) + "\tRGB[" + String(red_light) + "," + String(green_light) + "," + String(blue_light) + "]";
      
      disableSensorAPDS();
      return dataAPDS;
    }
  }else{
    Serial.println(F("Algo deu errado durante a inicialização dos sensores de Luz!"));
  }
  disableSensorAPDS();
  return "";
}

boolean configSensorAPDS() {
  // Inicializando sensor
  Wire.begin(); // Inicializando a comunicação I2C

  apds.init();
  
  if (apds.enableLightSensor(true)) {
      Serial.println(F("Sensor de Luz inicializado!"));
      return true;
  }else{
    return false;
  }
}


boolean configSensorTSL(){
  // Inicializando sensor
  Wire.begin();//inicializando  a comunicação I2C 
  
  if (tsl.begin()) {
    tsl.enableAutoRange(true); /* Ganho automático ... alterna automaticamente entre 1x e 16x */
    
    /* Alterar o tempo de integração proporciona uma melhor resolução do sensor (402ms = dados de 16 bits) */
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); 
    return true; // Retorna true se a inicialização foi bem-sucedida
  } else {
    return false; // Retorna false em caso de falha na inicialização
  }
}

void disableSensorAPDS(){
  apds.enableLightSensor(false);
  
  Wire.end();
}

void disableSensorTSL(){
  Wire.end();
}
