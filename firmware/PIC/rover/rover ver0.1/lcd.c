#include "lcd.h"
#include "config.h"

static void intro(void);

void writeDigit(unsigned char n)
{
  // Setup che dovrebbe essere di 10ns
  lcdE = 1;

  // Mette il digit nelle linee da D4 a D7
  if (n & 0x01) lcdD4 = 1; else lcdD4 = 0;
  if (n & 0x02) lcdD5 = 1; else lcdD5 = 0;
  if (n & 0x04) lcdD6 = 1; else lcdD6 = 0;
  if (n & 0x08) lcdD7 = 1; else lcdD7 = 0;

  //Hold time sicuramente superiore a 80ns.

  lcdE = 0;
}

void writeByte(unsigned char n)
{
  // Prima invia la parte alta
  writeDigit(n >> 4);
  // Dopo invia la parte bassa
  writeDigit(n & 0x0F);
}

void lcdWriteCmd(unsigned char n)
{
  // Per inviare un comando la linea RS deve essere posta a 0
  lcdRS = 0;
  // Invia il dato
  writeByte(n);
  // Ritarda per attendere la completa esecuzione del comando
  // In questo caso dipende dal comando
  // questi ritardi sono di 1600 e 43 us massimi
  if (!(n & 0xFC)) for(int i = 0; i < 100; i++) __delay_us(16);
  else __delay_us(43);
}

void lcdWriteChar(unsigned char n)
{
  // Per inviare un comando la linea RS deve essere posta a 0
  lcdRS = 1;
  // Invia il dato
  writeByte(n);
  // Ritarda per attendere la completa esecuzione del comando
  // anche questo ritardo deve essere 43us
  __delay_us(43);
}

void lcdInit(void)
{
  // inizializza le linee di uscita
  lcdD4_DIR = 0;
  lcdD5_DIR = 0;
  lcdD6_DIR = 0;
  lcdD7_DIR = 0;

  lcdRS_DIR = 0;
  lcdRS = 0;

  lcdE_DIR = 0;
  lcdE = 0;

  // Ritardo di 15ms per stabilizzazione VCC
  __delay_ms(15);
  // esegue la sequenza di reset, quella che si usa in caso di alimentazione
  // che non garantisce un buon reset interno.

  lcdRS = 0;           // mette RS a 0
  writeDigit(0x03);
  for(int i = 0; i < 100; i++) __delay_us(41);  //sul datasheet e' indicato 4,1 ms

  writeDigit(0x03);
  for(int i = 0; i < 10; i++) __delay_us(12);// 120us. Il datasheet indica > 100 us.

  writeDigit(0x03);
  for(int i = 0; i < 10; i++) __delay_us(12);

  writeDigit(0x02);
  for(int i = 0; i < 10; i++) __delay_us(12);
  // Fine sequenza di reset

  // Programma il controller con il numero di righe da pilotare (1 o 2)
  // Data lenght 4 bit e font 5x8
  #if (1 == lcdLINES)
    lcdWriteCmd(0x20);
  #elif (2 == lcdLINES)
    lcdWriteCmd(0x28);
  #else
    errore [HD44780] numero di righe non supportato.
  #endif

  // Cancella il display
  lcdWriteCmd(0x01);

  // Imposta direzione d' incremento in avanti senza shift del display
  lcdWriteCmd(0x06);

  // lcd acceso e cursore lampeggiante a linea non visibile
  lcdWriteCmd(0x0C);

  intro();
}

//******************************//
void lcdClear(void)
{
  lcdWriteCmd(lcdClear_CMD);
}

void lcdSetPos(unsigned char x, unsigned char y)
{
  unsigned char p;

  switch(y)
  {
    case 0: p = x; break;
    case 1: p = 64 + x; break;
    case 2: p = 20 + x; break;
    case 3: p = 84 + x; break;
  }
  p |= lcdSetADR_CMD;
  lcdWriteCmd(p);
}

void lcdWriteStrC(const char* s)
{
  while(*s) lcdWriteChar(*s++);
}

void lcdWriteStr(char* s)
{
  while(*s) lcdWriteChar(*s++);
}
void lcdClean()
{
    int i;
    for(i = 0; i < 16; i++)
        {
            lcdSetPos(i,0);
            lcdWriteChar('-');
            __delay_ms(20);

        }
        for(i = 0; i < 16; i++)
        {
            lcdSetPos(i,0);
            lcdWriteChar(' ');
            lcdSetPos(16-i,1);
            lcdWriteChar('-');
            __delay_ms(20);

        }
        for(i = 0; i < 16; i++)
        {
            lcdSetPos(i,1);
            lcdWriteChar(' ');
            __delay_ms(20);

        }
    lcdClear();
}

static void intro(void)
{
    // introduction
    int i;
    lcdSetPos(0,0);
    lcdWriteStrC("Roberino");
    lcdSetPos(0,1);
    lcdWriteStrC("Singh Be Rosa");

    for(i = 0; i < 100; i++) __delay_ms(50);
    lcdClean();

}