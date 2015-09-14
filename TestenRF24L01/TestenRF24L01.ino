#include <SPI.h>

// Conexoes
const int pinLED = 6;
const int pinBotao = 7;
const int pinSelAddr = 8;
const int pinCE = 9;
const int pinCSN = 10;

// Velocidade do SPI
const uint32_t RF_SPI_SPEED = 10000000;

// Comandos do nRF24L01+
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

// Registradores do nRF24L01+
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

// Bits do registrador de Status
#define RX_DR       6    // data ready
#define TX_DS       5    // data sent
#define MAX_RT      4    // max retransmissions

// Bits do registrador CONFIG
#define PRIM_RX     0
#define PWR_UP      1

uint8_t botaoAnt = HIGH;
uint8_t addrTx, addrRx;

void setup() 
{
  // Serial para debug
  Serial.begin(9600);

  // Iniciacao dos pinos de LED, Botao e Selecao do endereco
  pinMode (pinLED, OUTPUT);
  pinMode (pinBotao, INPUT);
  digitalWrite (pinBotao, HIGH);
  pinMode (pinSelAddr, INPUT);
  digitalWrite (pinSelAddr, HIGH);

  // Seleciona os enderecos
  if (digitalRead(pinSelAddr) == LOW)
  {
    addrRx = 1;
    addrTx = 2;
  }
  else
  {
    addrRx = 2;
    addrTx = 1;
  }
  Serial.print("Recebendo no endereco ");
  Serial.println(addrRx);

  // Iniciacao do Radio
  radioInit();
  startRx();
}

void loop() 
{
  uint8_t botao;
  
  botao = digitalRead(pinBotao);
  if (botao != botaoAnt)
  {
    botaoAnt = botao;
    Serial.print ("Transmitindo: ");
    stopRx();
    if (txDado (botao))
       Serial.println ("Ok");
    else    
       Serial.println ("Erro");
    startRx();
  }
  else if (temRx())
  {
    uint8_t dado = rxDado();
    Serial.print("Recebido ");
    Serial.println(dado);
    if (dado == LOW)
      digitalWrite(pinLED, HIGH);
    else
      digitalWrite(pinLED, LOW);
  }
}

// Iniciacao do Radio
void radioInit()
{
  uint8_t addr[3];
  
  Serial.println ("Inciando radio...");
  
  // Incia o SPI
  SPI.begin();
  
  // Inicia os sinais de controle
  pinMode (pinCE, OUTPUT);
  digitalWrite(pinCE, LOW);
  pinMode (pinCSN, OUTPUT);
  digitalWrite(pinCSN, HIGH);

  // Configura o radio
  delay(5);  // para o caso do radio acabar de ser ligado
  write_register(CONFIG, 0b00001100);  // CRC de 16 bits
  write_register(SETUP_RETR, 0x5F);// ate 15 retries, timeout = 1,5ms
  write_register(RF_SETUP, 0x06);  // 1Mbps, Potencia maxima
  write_register(FEATURE,0 );      // trabalhar com pacotes de tamanho fixo
  write_register(DYNPD,0);         // trabalhar com pacotes de tamanho fixo
  write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
  write_register(RF_CH, 76);       // usar canal 76
  send_command(FLUSH_RX);          // limpa a recepcao
  send_command(FLUSH_TX);          // limpa a transmissao
  write_register(CONFIG, read_register(CONFIG) | _BV(PRIM_RX));  // modo Rx
  powerUp();                       // liga o radio

  // Configura os enderecos  
  addr[0] = addrTx;
  addr[1] = 0;
  addr[2] = 0;
  write_register(SETUP_AW, 1);          // enderecos de 3 bytes
  writeN_register(TX_ADDR, addr, 3);    // endereco de transmissao
  writeN_register(RX_ADDR_P0, addr, 3); // auto ACK
  addr[0] = addrRx;
  writeN_register(RX_ADDR_P1, addr,3);  // endereco de recepcao
  write_register(EN_RXADDR,2);          // recepcao habilitada no pipe 1

  //  Pacotes com 1 byte de dado
  write_register(RX_PW_P1, 1);
  
  Serial.println ("Radio iniciado.");
}

// Envia um byte
uint8_t txDado (uint8_t dado)
{
  uint8_t status;
  
  // Coloca na fila o byte a transmitir
  digitalWrite(pinCSN, LOW);
  SPI.transfer(W_TX_PAYLOAD);
  SPI.transfer(dado);
  digitalWrite(pinCSN, HIGH);
  
  // Dispara a transmissao
  digitalWrite(pinCE, HIGH);
  
  // Espera concluir
  while ((read_register(NRF_STATUS) & (_BV(TX_DS) | _BV(MAX_RT))) == 0)
    ;
    
  // Desligar o transmissor
  digitalWrite(pinCE, LOW);

  // desliga recepcao no pipe 0
  write_register(EN_RXADDR,2);  
  
  // Verifica o resultado
  status = write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
  if( status & _BV(MAX_RT))
  {
    send_command(FLUSH_TX);
    return 0;
  }
  else
  {
    return 1;
  }
}

// Inicia a recepcao
void startRx(void)
{
  write_register(CONFIG, read_register(CONFIG) | _BV(PRIM_RX));
  write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
  digitalWrite(pinCE, HIGH);
}

// Para a recepcao
void stopRx(void)
{
  // Desliga RX
  digitalWrite(pinCE, LOW);
  delayMicroseconds(65);   // Tempo de recepcao
  delayMicroseconds(65);   // Tempo de envio de ACK
  send_command(FLUSH_TX);  // Limpa eventual ACK
  write_register(CONFIG, (read_register(CONFIG) ) & ~_BV(PRIM_RX));
  
  // Habilita recepcao no pipe 0 (para ACK)
  write_register(EN_RXADDR,read_register(EN_RXADDR) | 1);
}

// Testa se tem recepcao
uint8_t temRx(void)
{
  return read_register(NRF_STATUS) & _BV(RX_DR);
}

// Le o dado recebido e limpa flag de recebido
uint8_t rxDado(void)
{
  uint8_t dado;
  
  digitalWrite(pinCSN, LOW);
  SPI.transfer(R_RX_PAYLOAD);
  dado = SPI.transfer(0xFF);
  digitalWrite(pinCSN, HIGH);
  write_register(NRF_STATUS, _BV(RX_DR));
  return dado;  
}

// Liga o radio
void powerUp(void)
{
   uint8_t cfg = read_register(CONFIG);

   // Se estava desligado, liga e espera iniciar
   if (!(cfg & _BV(PWR_UP)))
   {
      write_register(CONFIG, cfg | _BV(PWR_UP));
      delay(5);
   }
}

// Envia comando ao nRF24L01+
uint8_t send_command(uint8_t cmd)
{
  uint8_t status;

  digitalWrite(pinCSN, LOW);
  status = SPI.transfer(cmd);
  digitalWrite(pinCSN, HIGH);
  return status;
}


// Escreve um valor em um registrador do nRF24L01+
uint8_t write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  digitalWrite(pinCSN, LOW);
  status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI.transfer(value);
  digitalWrite(pinCSN, HIGH);
  return status;
}

// Escreve varios valores em um registrador do nRF24L01+
uint8_t writeN_register(uint8_t reg, uint8_t *pValue, uint8_t n)
{
  uint8_t status;

  digitalWrite(pinCSN, LOW);
  status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  while (n--)
    SPI.transfer(*pValue++);
  digitalWrite(pinCSN, HIGH);
  return status;
}

// Le um registrador do nRF24L01+
uint8_t read_register(uint8_t reg)
{
  uint8_t result;

  digitalWrite(pinCSN, LOW);
  SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  result = SPI.transfer(0xff);
  digitalWrite(pinCSN, HIGH);
  #ifdef TRACE
  Serial.print ("Reg ");
  Serial.print (reg, HEX);
  Serial.print (" = ");
  Serial.println (result, HEX);
  #endif
  return result;  
}


