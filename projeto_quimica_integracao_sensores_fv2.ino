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

String dados, uni, value,rgbData;
char espaco;
int red,green,blue,rgbDataInt,ldr;

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

   Serial.println("Unidades de medições dos sensores disponiveis: 'APDS', 'TSL', 'RGB' e 'LDR'");
}


void loop(){
  if(Serial.available() > 0){
    dados = Serial.readStringUntil('\n');//Lendo inputs da porta serial

    espaco = dados.indexOf(' ');//quebrando a string até o espaço ' '

    value = dados.substring(0, espaco);//valor numerico da strind 
    uni = dados.substring(espaco+1);//valor da unidade da string(+1 carrega na variavel a parti do caractere " "(espaço)+1 da string, até o final

    //Indentificando função
    if(uni == "ldr" || uni == "LDR"){
      Serial.print(uni + ":" + "\t" + value + "\t");
      readSensorLDR();
    }else if(uni == "rgb" || uni == "RGB"){
      int dataRGB = readSensorRGB();

      while(dataRGB <= 0){
        dataRGB = readSensorRGB();
        delay(100);
      }

      Serial.print(uni + ":" + "\t" + value + "\t");
      Serial.println(String(dataRGB));
      
    }else if(uni == "tsl" || uni == "TSL"){
      Serial.print(uni + ":" + "\t" + value + "\t");
      readSensorTSL();
    }else if(uni == "apds" || uni == "APDS"){
      Serial.print(uni + ":" + "\t" + value + "\t");
      readSensorAPDS();
    }else{
      Serial.println("Input invalido");
    }
    delay(1000);
  }  
}

void readSensorLDR(){
  ldr = analogRead(A0);

  Serial.println(ldr);
}

int readSensorRGB(){
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
  return rgbString.toInt();
}

void readSensorTSL(){
  //Config:
  if(configSensorTSL() == true){
      sensors_event_t event;//variavel do tipo sensors_event_t    
      /*sensors_event_t: estrutura de dados que é usada para armazenar os resultados da leitura do sensor.*/
      
      tsl.getEvent(&event);//lendo dados do sensor e atualizando na variavel event
     
      /* Exibir os resultados (a luz é medida em lux) */
      if (event.light){
        Serial.print(event.light); Serial.println(" lux");
      }
      else{
        /* Se event.light = 0 lux, o sensor está provavelmente saturado
           e nenhum dado confiável pôde ser gerado! */
        Serial.println("Sobrecarga do sensor");
      }
    
  }else{
    Serial.println("Erro na inicialização do sensor TSL2561!");
  }

  disableSensorTSL();
}

void readSensorAPDS(){
  //Config:
  if(configSensorAPDS() == true){
          if (  !apds.readAmbientLight(ambient_light) ||
          !apds.readRedLight(red_light) ||
          !apds.readGreenLight(green_light) ||
          !apds.readBlueLight(blue_light) ) {
      Serial.println("Erro ao ler os valores de luz");
    } else {
      Serial.print("Ambiente: ");
      Serial.print(ambient_light);
      Serial.print(" Vermelha: ");
      Serial.print(red_light);
      Serial.print(" Verde: ");
      Serial.print(green_light);
      Serial.print(" Azul: ");
      Serial.println(blue_light);
    }
  }else{
    Serial.println("Algo deu errado durante a inicialização dos sensores de Luz!");
  }
  disableSensorAPDS();
}

boolean configSensorAPDS() {
  // Inicializando sensor
  Wire.begin(); // Inicializando a comunicação I2C

  apds.init();
  
  if (apds.enableLightSensor(true)) {
      Serial.println("Sensor de Luz inicializado!");
      return true;
  }else{
    return false;
  }
  /*ANTIGO CÓDIGO:
  if (apds.init()) {
    Serial.println("O sensor está funcionando agora");

    apds.enableGestureSensor(false);     // Desativando Sensor de Gestos
    apds.enableProximitySensor(false);   // Desativando Sensor de Proximidade

    //Erro: essa biblioteca não dá suporte ao sensor de luz ou está com problemas
    if (apds.enableLightSensor(true)) {
      Serial.println("Sensor de Luz inicializado!");
      return true;
    }
  } else {
    Serial.println("Algo deu errado durante a inicialização do sensor");
  }
  return false;  // Retorna falso se a inicialização falhar em qualquer ponto*/
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
