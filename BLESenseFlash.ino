/*
 * BLESenseFlash.ino
 * 
 * author: tom@tjpetz.com
 * 
 * This is a simple sketch to experiment with writting to and erasing the flash memory on the Arduino Nano BLE Sense.
 * The board is nRF52 based.  The arduino code runs on top of mbed so it's probably possible to use some of the 
 * underlying mbed api's to manipulate the flash memory.  In this sketch we're manipulating the flash very bare
 * bones via the NVMC controller on the chip.
 * 
 * The basics of this code are as follows:
 *  - understand how the various __atrribute__ settings affect the location of the data in memory and the
 *    alignment of the memory.
 *  - the first set of variables that are not const are all located within RAM and are page aligned.  While 
 *    they are put in the section FLASH, unless we manipulate the linker settings this doesn't really have
 *    much of a useful effect in our code.
 *  - the set of variables declared const are the key variables.  As they are const the linker places these   
 *    in flash.  They are initialized and those initial values are writting into the flash when the sketch
 *    is loaded onto the device.
 *  - when writing to flash we need to confirm the NVMC controller is ready. We then set the config register 
 *    to put it in write mode.  We write the data and then must wait for the write to complete.  The check
 *    needs to be in a tight loop as the NVMC controller needs not to be disturbed during the write so the 
 *    loop needs to fit in the cache.  When writting the max time to write to a word to flash is 41 uS.
 *  - When erasing the flash it will initialize the page to all 1's. Note it can take 80 mS per the spec 
 *    to erase a page.
 *  - alignment and manage of the variables is very important.  We need to work with the variables as a
 *    whole page because we must erase the page to set any bit within the page to 1.  The NVMC controller
 *    can only zero a bit and must also always write a word at a time.  Therefore when using this code
 *    in a real situation the struct should be designed to be page aligned and should be sized to consume
 *    the whole page.  This will prevent code or other const variables from being accidentally erased.
 *    
 * When the code executes the very first time the initial values of myflash variable have been initialized   
 * with the struct values.  These get programmed by the loader when loading the sketch on to the board.
 * The code then dumps the memory to show that the contents are expected.  The first member of the struct
 * is then zero'd out.  And we dump the memory again to confirm only the first word of the struct has 
 * been changed.  We then erase the whole page to confirm an erase works.  Finally we dump the contents
 * of flash2, another item in the flash on a different page, to confirm it's not been erased.
 * 
 * A use case for this code is to manage a set of user configuration settings.  The sketch can load a
 * reasonable set of defaults.  Then the user would update the defaults with BLE.  Once the defaults are
 * changed they would survive a board reboot.  (Note, however, reloading the sketch from the programmer
 * would reset the flash to the default values.)
 * 
 */
 
#include <Arduino.h>

// nFR52 NVMC registers
#define NVMC_BASE       (0x4001E000U)
#define NVMC_READY      (NVMC_BASE + 0x400U)
#define NVMC_READYNEXT  (NVMC_BASE + 0x408U)
#define NVMC_CONFIG     (NVMC_BASE + 0x504U)
#define NVMC_ERASEPAGE  (NVMC_BASE + 0x508U)
#define NVMC_ERASEALL   (NVMC_BASE + 0x50CU)
#define NVMC_ERASEUICR  (NVMC_BASE + 0x514U)
#define NVMC_ERASEPAGEPARTIAL   (NVMC_BASE + 0X518U)
#define NVMC_ERASEPAGEPARTIALCFG  (NVMC_BASE + 0X51CU)
#define NVMC_ICACHECNF  (NVMC_BASE + 0x540U)
#define NVMC_IHIT       (NVMC_BASE + 0x548U)
#define NVMC_IMISS      (NMVC_BASE + 0x54cU)


// lets put some memory in a specific location.  We'll put it in a specific section as we explore
// how the linker might manage it and move it about.

byte page1 [0x1000] __attribute__ ((section("FLASH"), aligned (0x1000)));
byte page2 [255] __attribute__ ((section("FLASH")));
byte page2a [255] __attribute__((section("FLASH")));
byte page3 [0x1000] __attribute__ ((section("FLASH"), aligned (0x1000)));

// From this experiment we learn that the variables must be const.  And when const the linker will
// automatically put them in flash.  This way we do not need to worry about the specific layout
// in memory as the linker will handle that automatically for us.

const int flash __attribute__ ((section("FLASH2"), aligned (0x1000))) = 100;
const uint32_t flash2 __attribute__ ((section("FLASH2"), aligned (0x1000))) = 0x09090909;

typedef struct flash_mem {
    char ssid[64];
    char pwd[64];
    char devname[64];
} flash_mem_t;

const flash_mem_t myFlash __attribute__ ((section("FLASH2"), aligned (0x1000))) = {
  "test", "secret", "fred"
}; 

// These are available for the linker and help us understand how memory is getting layed out.
extern "C" unsigned long __heap_start;
extern "C" unsigned long __heap_size;
extern "C" unsigned long __bss_start__;
extern "C" unsigned long __bss_end__;
extern "C" unsigned long FLASH;     // While available this always ends up being zero, showing us that
                                    // we can't get the link address location this way from the linker.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2500);
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Starting...");
  Serial.print("__heap_start: "); Serial.println(__heap_start, HEX);
  Serial.print("__heap_size: "); Serial.println(__heap_size, HEX);
  Serial.print("&page1: "); Serial.println((unsigned long)(&page1), HEX);
  Serial.print("&page2: "); Serial.println((unsigned long)(&page2), HEX);
  Serial.print("&page2a: "); Serial.println((unsigned long)(&page2a), HEX);
  Serial.print("&page3: "); Serial.println((unsigned long)(&page3), HEX);
  Serial.print("&flash: "); Serial.println((unsigned long)(&flash), HEX);
  Serial.print("&myFlash: "); Serial.println((unsigned long)(&myFlash), HEX);
  Serial.print("__bss_start__: "); Serial.println(__bss_start__, HEX);
  Serial.print("__bss_end__: "); Serial.println(__bss_end__, HEX);
  Serial.print("FLASH: "); Serial.println(FLASH, HEX);
  Serial.flush();

  Serial.print("NVMC READY: "); Serial.println(*(uint32_t *)(NVMC_READY), HEX);
  Serial.print("NVMC READY NEXT: "); Serial.println(*(uint32_t *)(NVMC_READYNEXT), HEX);
  
  hexDumpMemory((uint8_t *)(&flash2), 64);
  hexDumpMemory((uint8_t *)(&myFlash), 256);

  Serial.print("Config = "); Serial.println(*(int32_t *)(NVMC_CONFIG), HEX);

  if (*(uint32_t *)NVMC_READY == 1) {
    Serial.println("Flashing...");
    // Write into myFlash
     *(uint32_t *)NVMC_CONFIG = 0x01;
    *(uint32_t *)(&myFlash) = 0x00;
    while(*(uint32_t *)NVMC_READY == 0)
      delayMicroseconds(50);
    *(uint32_t *)NVMC_CONFIG = 0x00;
    Serial.println("...Flashed");
  } else {
    Serial.println("... Flash Not Ready! ...");
  }
  
  hexDumpMemory((uint8_t *)(&myFlash), 256);

  if(*(uint32_t *)NVMC_READY == 1) {
    Serial.println("Erasing...");
    *(uint32_t *)NVMC_CONFIG = 0x02;
    *(uint32_t *)NVMC_ERASEPAGE = (uint32_t)(&myFlash);
    while(*(uint32_t *)NVMC_READY == 0)
      delay(85);
    *(uint32_t *)NVMC_CONFIG = 0x00;
    Serial.println("...Erased");
  } else {
    Serial.println("... Flash Not Ready to Erase! ...");
  }

  hexDumpMemory((uint8_t *)(&myFlash), 256);
  hexDumpMemory((uint8_t *)(&flash2), 64);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(750);
}

void hexDumpMemory(uint8_t *memStart, const unsigned int nbrOfBytes) {
  /* hex dump memory to the serial interface starting at memStart for nbrOfBytes */

  uint8_t *ptr;
  int bytesPerLine = 15;
  int bytesOnLine = 0;
  
  ptr = memStart;

  Serial.print("Memory dump of: "); Serial.println((unsigned long)(memStart), HEX);
  
  for (unsigned int i = 0; i < nbrOfBytes; i++) {
    if (bytesOnLine == 0) {
      Serial.print((unsigned long)(ptr+i), HEX);
      Serial.print(": ");
    }
    
    if (*(ptr+i) < 0x10)  // print a leading 0
      Serial.print("0");
    Serial.print(*(ptr+i), HEX); Serial.print(" ");
    
    if (bytesOnLine == bytesPerLine) {
      Serial.println(" ");
      bytesOnLine = 0;
    } else {
      bytesOnLine += 1;
    }
  }
  Serial.println("");
}
