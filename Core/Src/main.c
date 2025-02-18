/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * Copyright (c) 2025 Mateusz Mroczkowski
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_TX_BUFFER_SIZE 1512
#define USART_RX_BUFFER_SIZE 533
#define FRAME_START_CHAR ':'
#define FRAME_END_CHAR ';'
#define FRAME_MAX_LENGTH 271
#define FRAME_ESCAPED_MAX_LENGTH 533
#define FRAME_FIXED_SIZE 15
#define SENDER_LENGTH 3
#define RECEIVER_LENGTH 3
#define CMD_LENGTH_DIGITS 3
#define CRC_LENGTH 4
#define DEVICE_ID "STM"
#define TERMINAL "TERM"

#define ADC_BUFFER_SIZE 1000
#define ADC_HALF_BUFFER_SIZE (ADC_BUFFER_SIZE/2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t usartTransmitBuffer[USART_TX_BUFFER_SIZE];  // Bufor do wysyłania
volatile uint16_t usartTransmitWriteIndex;  // Gdzie zapisujemy nowe dane do wysłania
volatile uint16_t usartTransmitReadIndex;   // Skąd odczytujemy dane do wysłania

uint8_t usartReceiveBuffer[USART_RX_BUFFER_SIZE];  // Bufor na odebrane dane
volatile uint16_t usartReceiveWriteIndex;  // Gdzie zapisujemy odebrane dane
volatile uint16_t usartReceiveReadIndex;   // Skąd odczytujemy odebrane dane

uint16_t adcBuffer[ADC_BUFFER_SIZE];  // Bufor na dane z ADC
volatile uint8_t adcBufferHalfReady = 0;  // Flaga pierwszej połowy bufora
volatile uint8_t adcBufferFullReady = 0;  // Flaga drugiej połowy bufora

FATFS FatFs;  // struktura do systemu plików FAT
FIL fil;      // do operacji na pojedynczych plikach
FRESULT fres; // do wyników operacji

volatile uint8_t fileOpenForWrite = 0;  // flaga pokazująca czy plik jest otwarty do zapisywania
FIL writeFile;                          // do operacji zapisu
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t calculateCRC16(const char* data, uint16_t length);
void sendFormattedUsart(char* format, ...);
uint16_t addEscapedChar(char* buffer, uint16_t position, char character);
void sendFrameResponse(const char* data, uint16_t dataLength, const char* receiver);
uint16_t calculateMaxEscapedLength(const char* data, uint16_t dataLength, const char* sender, const char* receiver);
void handleGetCommand(const char* sender);
uint8_t checkAndAddToBuffer(uint8_t* buffer, uint16_t* index, uint8_t* frameFound, uint8_t data);
uint8_t isValidName(const char* name);
void handleShowStatus(const char* sender);
void handleListCommand(const char* data, uint16_t dataLen, const char* sender);
void handleMkdirCommand(const char* data, uint16_t dataLen, const char* sender);
uint16_t convertCmdLength(const char* lengthStr);
void handleRemoveCommand(const char* data, uint16_t dataLen, const char* sender);
void handleMkfileCommand(const char* data, uint16_t dataLen, const char* sender);
void handleReadCommand(const char* data, uint16_t dataLen, const char* sender);
void handleWriteCommand(const char* data, uint16_t dataLen, const char* sender);
void handleSendCommand(const char* data, uint16_t dataLen, const char* sender);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
    char sender[SENDER_LENGTH + 1];        // 3 chars + null terminator
    char receiver[RECEIVER_LENGTH + 1];    // 3 chars + null terminator
    char cmdLength[CMD_LENGTH_DIGITS + 1]; // 3 digits + null terminator
    char* data;                            // Dane (dynamicznie alokowany)
    char crc[CRC_LENGTH + 1];              // 4 hex + null terminator
} Frame;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	// Sprawdza czy przerwanie dotyczy właściwego UART-a
    if(huart == &huart2) {
    	// Sprawdzenie czy są dane do wysłania
    	// Jeśli indeksy są różne, oznacza to że w buforze są jeszcze dane do wysłania
        if(usartTransmitWriteIndex != usartTransmitReadIndex) {
        	// Pobranie następnego znaku
            uint8_t nextChar = usartTransmitBuffer[usartTransmitReadIndex];
            // Zwiększa indeks odczytu
            usartTransmitReadIndex++;
            // Jeśli indeks doszedł do końca bufora, wraca na początek
            if(usartTransmitReadIndex >= USART_TX_BUFFER_SIZE) {
                usartTransmitReadIndex = 0;
            }
            // Wysyłanie znaku
            HAL_UART_Transmit_IT(&huart2, &nextChar, 1);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//Sprawdza czy przerwanie dotyczy właściwego UART-a
    if(huart == &huart2) {
    	// Zwiększa indeks zapisu dla kolejnego znaku
        usartReceiveWriteIndex++;
        // Jeśli indeks osiągnie koniec bufora, wraca na początek
        if(usartReceiveWriteIndex >= USART_RX_BUFFER_SIZE) {
            usartReceiveWriteIndex = 0;
        }
        // Uruchomienie odbioru następnego znaku
        HAL_UART_Receive_IT(&huart2, &usartReceiveBuffer[usartReceiveWriteIndex], 1);
    }
}

// Funkcja sprawdzająca czy w buforze odbiorczym (RX) są dostępne jakieś dane do odczytania:
uint8_t isUsartDataAvailable() {
    return (usartReceiveWriteIndex != usartReceiveReadIndex);
    // Zwraca 1 (true) gdy indeksy są różne - oznacza to że są nowe, nieodczytane dane w buforze
    // Zwraca 0 (false) gdy indeksy są równe - oznacza to że nie ma nowych danych
}

int16_t readUsartChar() {
    uint8_t receivedChar;
    if(isUsartDataAvailable()) {
    	// Odczytuje znak z bufora pod aktualnym indeksem odczytu
        receivedChar = usartReceiveBuffer[usartReceiveReadIndex];
        // Zwiększa indeks odczytu
        usartReceiveReadIndex++;
        // Jeśli indeks doszedł do końca bufora, wraca na początek
        if(usartReceiveReadIndex >= USART_RX_BUFFER_SIZE) {
            usartReceiveReadIndex = 0;
        }
        return receivedChar;
    }
    // Jeśli nie ma danych do odczytu, zwraca -1, sygnalizuje brak danych
    return -1;
}

// static zachowuje wartość między wywołaniami funkcji, Zmienna jest inicjalizowana tylko raz

uint16_t readUsartLine(char *outputBuffer) {
    static uint8_t lineBuffer[FRAME_MAX_LENGTH];
    static uint16_t bufferIndex = 0;
    static uint8_t frameStartFound = 0;
    static uint8_t escapeNext = 0;

    while(isUsartDataAvailable()) {
        int16_t receivedChar = readUsartChar();

        // Zawsze sprawdzamy początek ramki
        if(receivedChar == FRAME_START_CHAR) {
            bufferIndex = 0;
            frameStartFound = 1;
            escapeNext = 0;
            if(checkAndAddToBuffer(lineBuffer, &bufferIndex, &frameStartFound, receivedChar)) continue;
            continue;
        }

        if(!frameStartFound) {
            continue;
        }

        // Obsługa escape
        if(escapeNext) {
            char actualChar;
            switch(receivedChar) {
                case '1': actualChar = ':'; break;
                case '2': actualChar = ';'; break;
                case '3': actualChar = '/'; break;
                default:
                    frameStartFound = 0;  // Nieprawidłowa sekwencja escape
                    escapeNext = 0;
                    continue;
            }
            if(checkAndAddToBuffer(lineBuffer, &bufferIndex, &frameStartFound, actualChar)) continue;
            escapeNext = 0;
            continue;
        }

        if(receivedChar == '/') {
            escapeNext = 1;
            continue;
        }

        if(checkAndAddToBuffer(lineBuffer, &bufferIndex, &frameStartFound, receivedChar)) continue;

        if(receivedChar == FRAME_END_CHAR) {
            for(uint16_t i = 0; i < bufferIndex; i++) {
                outputBuffer[i] = lineBuffer[i];
            }
            uint16_t frameLength = bufferIndex;
            bufferIndex = 0;
            frameStartFound = 0;
            escapeNext = 0;
            return frameLength;
        }
    }

    return 0;
}

uint8_t checkAndAddToBuffer(uint8_t* buffer, uint16_t* index, uint8_t* frameFound, uint8_t data) {
    buffer[(*index)++] = data;
    if(*index >= FRAME_MAX_LENGTH) {
        *index = 0;
        *frameFound = 0;
        sendFrameResponse("\r\nERR: Buffer overflow\r\n", 24, TERMINAL);
        return 1;
    }
    return 0;
}

// Służy do formatowania i wysyłania danych przez UART, ... oznacza że funkcja może przyjąć zmienną liczbę argumentów
void sendFormattedUsart(char* format, ...) {
	// Tymczasowa tablica do przechowania
    char tempString[USART_TX_BUFFER_SIZE];
    uint16_t i;
    __IO uint16_t writeIndex;
    va_list arguments;
    uint16_t stringLength;

    // Inicjalizuje listę argumentów
    va_start(arguments, format);
    // Formatuje tekst według zadanego formatu i zapisuje go do tempString,
    // vsprintf zwraca liczbę zapisanych znaków (bez null terminatora)
    stringLength = vsprintf(tempString, format, arguments);
    // Kończy pracę z listą argumentów
    va_end(arguments);

    writeIndex = usartTransmitWriteIndex;

    // Kopiuje sformatowany tekst znak po znaku do bufora cyklicznego usartTransmitBuffer
    for(i = 0; i < stringLength; i++) {
        usartTransmitBuffer[writeIndex] = tempString[i];
        writeIndex++;
        // Jeśli dojdzie do końca bufora, wraca na początek
        if(writeIndex >= USART_TX_BUFFER_SIZE) {
            writeIndex = 0;
        }
    }
    // Wyłącza przerwania
    __disable_irq();

    // UART_FLAG_TXE oznacza UART Transmit Data Register Empty
    // Warunek sprawdza czy rejestr wysyłania jest pusty, nie wysyła aktualnie danych i jest gotowy do wysyłania
    if((usartTransmitWriteIndex == usartTransmitReadIndex) &&
       (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
    	// Aktualizuje główny indeks zapisu nowymi danymi
        usartTransmitWriteIndex = writeIndex;
        // Pobiera pierwszy znak do wysłania
        uint8_t nextChar = usartTransmitBuffer[usartTransmitReadIndex];
        // Przesuwa indeks odczytu na następną pozycję
        usartTransmitReadIndex++;
        // Sprawdza czy indeks nie przekroczył rozmiaru bufora
        if(usartTransmitReadIndex >= USART_TX_BUFFER_SIZE) {
            usartTransmitReadIndex = 0;
        }
        // Rozpoczyna transmisję pierwszego znaku
        HAL_UART_Transmit_IT(&huart2, &nextChar, 1);
    } else {
    	 // Jeśli transmisja jest w toku lub UART nie jest gotowy, tylko aktualizuje indeks zapisu
        usartTransmitWriteIndex = writeIndex;
    }
    // Włącza z powrotem przerwania po zakończeniu operacji na współdzielonych zmiennych
    __enable_irq();
}

// const jest używane w tej funkcji do ochrony danych przed modyfikacją, można zmienić na co wskazuje ale nie zawartość

uint8_t parseFrame(const char* inputBuffer, uint16_t bufferLength, Frame* frame) {
	uint16_t i;
    const char* frameStart = NULL;  // wskazniki na znaki
    const char* frameEnd = NULL;
    uint16_t dataOffset;
    uint16_t dataLength;
    char frameWithoutCRC[FRAME_MAX_LENGTH];  // Tymczasowy bufor na ramkę bez CRC

    frameStart = &inputBuffer[0];  // Wiemy że to jest ':'
    frameEnd = &inputBuffer[bufferLength - 1];  // Wiemy że to jest ';'

    // Sprawdzenie czy ramka nie jest za długa
    if(bufferLength > FRAME_MAX_LENGTH) {
    	sendFrameResponse("\r\nERR: Frame too long\r\n", 23, TERMINAL);
        return 0;
    }

    // Sprawdzenie minimalnej długości (znak początku + nadawca + odbiorca + długość + CRC + znak końca)
    if(bufferLength < (1 + SENDER_LENGTH + RECEIVER_LENGTH + CMD_LENGTH_DIGITS + CRC_LENGTH + 1)) {
    	sendFrameResponse("\r\nERR: Frame too short\r\n", 24, TERMINAL);
        return 0;
    }

    // Wydobycie crc, zapis do frame, kopiujemy od konca -4, 4 bajty
    memcpy(frame->crc, frameEnd - CRC_LENGTH, CRC_LENGTH);
    frame->crc[CRC_LENGTH] = '\0';

    // Konwersja crc, |= or
    uint16_t receivedCRC = 0;
    for(i = 0; i < CRC_LENGTH; i++) {
        receivedCRC <<= CRC_LENGTH; // przesunięcie o 4 w lewo
        if(frame->crc[i] >= '0' && frame->crc[i] <= '9') {
            receivedCRC |= frame->crc[i] - '0'; // np. 2  czyli 50 - 48 = 2, 0010
        } else if(frame->crc[i] >= 'A' && frame->crc[i] <= 'F') {
            receivedCRC |= frame->crc[i] - 'A' + 10; // np. B czyli 66 - 65 + 10 = 11, 1011
        } else {
        	sendFrameResponse("\r\nERR: CRC conversion\r\n", 23, TERMINAL);
            return 0;
        }
    }

    // Oblicz CRC (bez pola CRC)
    uint16_t dataForCRC = frameEnd - frameStart - CRC_LENGTH;  // odleglosc miedzy wskaznikami, oraz -4
    memcpy(frameWithoutCRC, frameStart, dataForCRC);
    uint16_t calculatedCRC = calculateCRC16(frameWithoutCRC, dataForCRC);

    // Porownanie CRC
    if(calculatedCRC != receivedCRC) {
    	sendFrameResponse("\r\nERR: CRC is wrong\r\n", 21, TERMINAL);
        return 0;
    }

    // Przesunięcie wskaźnika za znak rozpoczynający
    frameStart++;

    // Ekstrakcja nadawcy
    memcpy(frame->sender, frameStart, SENDER_LENGTH);
    frame->sender[SENDER_LENGTH] = '\0';

    // Ekstrakcja odbiorcy
    memcpy(frame->receiver, frameStart + SENDER_LENGTH, RECEIVER_LENGTH);
    frame->receiver[RECEIVER_LENGTH] = '\0';

    // Sprawdzenie czy jesteśmy odbiorcą, zwraca 0 gdy sa takie same, (!=)
    if(memcmp(frame->receiver, DEVICE_ID, RECEIVER_LENGTH) != 0) {
    	sendFrameResponse("\r\nERR: Invalid receiver\r\n", 25, TERMINAL);
        return 0;
    }

    // Ekstrakcja długości komendy podanej przez użytkownika
    memcpy(frame->cmdLength, frameStart + SENDER_LENGTH + RECEIVER_LENGTH, CMD_LENGTH_DIGITS);
    frame->cmdLength[CMD_LENGTH_DIGITS] = '\0';


	// declaredDataLength = 0
	// sprawdź czy '1' jest cyfrą (jest między '0' a '9')
	// '1' - '0' = 1 (ASCII 49 - 48 = 1)
	// declaredDataLength = 0 * 10 + 1 = 1

    // Konwersja długości komendy podanej przez użytkownika na int
    uint16_t declaredDataLength = 0;
    for(i = 0; i < CMD_LENGTH_DIGITS; i++) {
        if(frame->cmdLength[i] < '0' || frame->cmdLength[i] > '9') {
        	sendFrameResponse("\r\nERR: Invalid command length format\r\n", 38, TERMINAL);
            return 0;
        }
        declaredDataLength = declaredDataLength * 10 + (frame->cmdLength[i] - '0');
    }

    // sprawdzenie dlugosci komendy podanej przez użytkownika
    if(declaredDataLength <= 0 || declaredDataLength > 256) {
    	sendFrameResponse("\r\nERR: Invalid command length\r\n", 31, TERMINAL);
        return 0;
    }

    // Oblicz pozycję początku danych i ich długość

    // wskazuje, że dane zaczynają się po 9 znakach od początku (po ':')
    dataOffset = SENDER_LENGTH + RECEIVER_LENGTH + CMD_LENGTH_DIGITS;
    dataLength = frameEnd - (frameStart + dataOffset) - CRC_LENGTH;  // -4

    // Sprawdzenie zgodności długości
    if(dataLength != declaredDataLength) {
    	sendFrameResponse("\r\nERR: Length mismatch\r\n", 24, TERMINAL);
        return 0;
    }

    // malloc() dynamicznie przydziela pamięć o rozmiarze cmdLen + 1 bajtów
	// +1 jest dodane aby zmieścić znak końca stringa '\0'
	// Zwraca wskaźnik do zaalokowanej pamięci
	// (char*) to rzutowanie (casting) zwróconego wskaźnika na typ char*
	// bo malloc zwraca ogólny wskaźnik void*
    // Jest to potrzebne bo frame->data jest zadeklarowane jako char*

    // Alokacja pamięci na dane
    frame->data = (char*)malloc(declaredDataLength + 1);
    // Sprawdza czy alokacja się udała, malloc() zwraca NULL jeśli nie może zaalokować pamięci
    if(!frame->data) {
    	sendFrameResponse("\r\nERR: Memory allocation failed\r\n", 33, TERMINAL);
        return 0;
    }

    // Kopiowanie danych
    memcpy(frame->data, frameStart + dataOffset, declaredDataLength);
    frame->data[declaredDataLength] = '\0';

    return 1;
}

void sendFrameResponse(const char* data, uint16_t dataLength, const char* receiver) {
    char escapedBuffer[FRAME_ESCAPED_MAX_LENGTH];  // bufor na dane escaped
    char unescapedBuffer[FRAME_MAX_LENGTH];    // bufor nie escaped do obliczenia crc
    uint16_t escapedLength = 0;
    uint16_t unescapedLength = 0;
    uint16_t i;

    uint16_t maxLength = calculateMaxEscapedLength(data, dataLength, DEVICE_ID, receiver);
    if(maxLength > FRAME_ESCAPED_MAX_LENGTH) {
        return;
    }

    // Start character w obu buforach
    escapedBuffer[escapedLength++] = FRAME_START_CHAR;
    unescapedBuffer[unescapedLength++] = FRAME_START_CHAR;

    // dodaj sender (DEVICE_ID)
	for(i = 0; i < SENDER_LENGTH; i++) {
		escapedLength = addEscapedChar(escapedBuffer, escapedLength, DEVICE_ID[i]);
		unescapedBuffer[unescapedLength++] = DEVICE_ID[i];
	}

	// dodaj receiver
	for(i = 0; i < RECEIVER_LENGTH; i++) {
		escapedLength = addEscapedChar(escapedBuffer, escapedLength, receiver[i]);
		unescapedBuffer[unescapedLength++] = receiver[i];
	}

    // makro zdefiniowane w nagłówku <inttypes.h>, które służy do zapewnienia przenośnego i bezpiecznego
    // formatowania liczb całkowitych o stałej szerokości, w tym przypadku typu uint16_t


	// %03 wymusza 3 cyfry z wiodącymi zerami

    // dodaj data length
    char lengthStr[4];
    snprintf(lengthStr, 4, "%03" PRIu16, dataLength);
    memcpy(escapedBuffer + escapedLength, lengthStr, 3);
    memcpy(unescapedBuffer + unescapedLength, lengthStr, 3);

    escapedLength += CMD_LENGTH_DIGITS;
    unescapedLength += CMD_LENGTH_DIGITS;

    // dodaj dane
	for(i = 0; i < dataLength; i++) {
		escapedLength = addEscapedChar(escapedBuffer, escapedLength, data[i]);
		unescapedBuffer[unescapedLength++] = data[i];
	}

    // oblicz CRC z unescaped buffer
    uint16_t crc = calculateCRC16(unescapedBuffer, unescapedLength);

    // dodaj crc
    snprintf(escapedBuffer + escapedLength, 5, "%04X", crc);
    escapedLength += CRC_LENGTH;

    // dodaj end character
    escapedBuffer[escapedLength++] = FRAME_END_CHAR;
    escapedBuffer[escapedLength] = '\0';

    sendFormattedUsart("%s", escapedBuffer);
}

uint16_t calculateMaxEscapedLength(const char* data, uint16_t dataLength, const char* sender, const char* receiver) {
    uint16_t maxLength = 1; // znak początku

    // sender
    for(uint8_t i = 0; i < SENDER_LENGTH; i++) {
        if(sender[i] == ':' || sender[i] == ';' || sender[i] == '/') {
            maxLength += 2;
        } else {
            maxLength += 1;
        }
    }

    // receiver
    for(uint8_t i = 0; i < RECEIVER_LENGTH; i++) {
        if(receiver[i] == ':' || receiver[i] == ';' || receiver[i] == '/') {
            maxLength += 2;
        } else {
            maxLength += 1;
        }
    }

    // command length
    maxLength += CMD_LENGTH_DIGITS;

    // data
    for(uint16_t i = 0; i < dataLength; i++) {
        if(data[i] == ':' || data[i] == ';' || data[i] == '/') {
            maxLength += 2;
        } else {
            maxLength += 1;
        }
    }

    // CRC
    maxLength += CRC_LENGTH;

    // znak konca
    maxLength += 1;

    return maxLength;
}

// Funkcja do dodawania znaków z escape
uint16_t addEscapedChar(char* buffer, uint16_t position, char character) {
    if(character == ':') {
        buffer[position++] = '/';
        buffer[position++] = '1';
    }
    else if(character == ';') {
        buffer[position++] = '/';
        buffer[position++] = '2';
    }
    else if(character == '/') {
        buffer[position++] = '/';
        buffer[position++] = '3';
    }
    else {
        buffer[position++] = character;
    }
    return position;
}

void processReceivedLine(char* inputBuffer, uint16_t bufferLength) {
	// utworzenie zmiennej frame
    Frame frame;

    //  wywołuje funkcję parseFrame
    //  &frame - adres utworzonej wcześniej struktury frame (gdzie zostaną zapisane sparsowane dane)
    if(!parseFrame(inputBuffer, bufferLength, &frame)) {
        return;
    }

    if(memcmp(frame.data, "led", 3) == 0) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        sendFrameResponse("LED state changed!", 18, frame.sender);
    }
    else if(memcmp(frame.data, "state", 5) == 0) {

    	// '0' w ASCII to 48
    	// Więc jeśli dioda wyłączona (0): 48 + 0 = 48 ('0')
    	// Jeśli dioda włączona (1): 48 + 1 = 49 ('1')
    	// Ten znak zastępuje '0' na pozycji msg[6]

    	char msg[] = "\r\nLED 0\r\n";
    	msg[6] = '0' + HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin);
    	sendFrameResponse(msg, 9, frame.sender);
    }
    else if(memcmp(frame.data, "get", 3) == 0) {
        handleGetCommand(frame.sender);
    }
    else if(memcmp(frame.data, "status", 6) == 0) {
        handleShowStatus(frame.sender);
    }
    else if(memcmp(frame.data, "ls", 2) == 0) {
    	handleListCommand(frame.data, convertCmdLength(frame.cmdLength), frame.sender);
    }
    else if(memcmp(frame.data, "mkdir", 5) == 0) {
    	handleMkdirCommand(frame.data, convertCmdLength(frame.cmdLength), frame.sender);
    }
    else if(memcmp(frame.data, "rm", 2) == 0) {
        handleRemoveCommand(frame.data, convertCmdLength(frame.cmdLength), frame.sender);
    }
    else if(memcmp(frame.data, "mkfile", 6) == 0) {
        handleMkfileCommand(frame.data, convertCmdLength(frame.cmdLength), frame.sender);
    }
    else if(memcmp(frame.data, "read", 4) == 0) {
        handleReadCommand(frame.data, convertCmdLength(frame.cmdLength), frame.sender);
    }
    else if(memcmp(frame.data, "write", 5) == 0) {
		handleWriteCommand(frame.data, convertCmdLength(frame.cmdLength), frame.sender);
	}
	else if(memcmp(frame.data, "send", 4) == 0) {
		handleSendCommand(frame.data, convertCmdLength(frame.cmdLength), frame.sender);
	}
    else {
    	sendFrameResponse("\r\nERR: Command error\r\n", 22, frame.sender);
    }

    if(frame.data) {
    	// Zwalnia zaalokowana pamiec dla danych
        free(frame.data);
    }
}

uint16_t calculateCRC16(const char* data, uint16_t length) {
    uint16_t crc = 0xFFFF;  // Wartość początkowa
    uint16_t poly = 0x1021; // Wielomian generujący x^16 + x^12 + x^5 + 1 czyli (ukryte 1) 0001 0000 0010 0001

    // data[i] - pobiera aktualny znak
    // (uint16_t)data[i] - konwertuje znak na 16-bitową wartość
    // << 8 - przesuwa wartość o 8 bitów w lewo (do górnego bajtu)
    // ^= - wykonuje operację XOR z aktualną wartością CRC ( 1 gdy 0,1 lub 1,0)

    // Przechodzi przez każdy znak w danych wejściowych
    for (uint16_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8); // Przesuń bajt danych do górnego bajtu

        // Wykonuje się 8 razy dla każdego bajtu
        // crc & 0x8000 - sprawdza najstarszy bit (MSB) rejestru CRC (operacja AND)
        // 0x8000 = 1000 0000 0000 0000 w binarnym

        // Jeśli MSB = 1
        // Przesuwa CRC o 1 bit w lewo
        // Wykonuje XOR z wielomianem (poly)

        // Jeśli MSB = 0
        // Tylko przesuwa CRC o 1 bit w lewo

        for (uint8_t j = 0; j < 8; j++) {
        	// Operacja AND (&) z maską 0x8000, sprawdza czy 1 czy 0
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {
        adcBufferHalfReady = 1;
        adcBufferFullReady = 0;  // Zeruj drugą flagę, bo teraz pierwsza połowa jest nowsza
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {
    	adcBufferHalfReady = 0; // Zeruj pierwszą flagę, bo teraz druga połowa jest nowsza
        adcBufferFullReady = 1;

        // Ponowne uruchomienie DMA po zakończeniu konwersji
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);
    }
}

// konwertuje ciąg znaków (string) reprezentujący długość komendy na liczbę całkowitą
uint16_t convertCmdLength(const char* lengthStr) {
    uint16_t length = 0;
    for(int i = 0; i < 3; i++) {
        if(lengthStr[i] >= '0' && lengthStr[i] <= '9') {
            length = length * 10 + (lengthStr[i] - '0');
        }
    }
    return length;
}

// Funkcja do obsługi komendy 'get' dla ADC
void handleGetCommand(const char* sender) {
    char response[24];
    uint16_t adcValue;

    // Pobieramy wartość ADC
    if(adcBufferFullReady) {
        adcValue = adcBuffer[ADC_BUFFER_SIZE - 1];
    }
    else if(adcBufferHalfReady) {
        adcValue = adcBuffer[ADC_HALF_BUFFER_SIZE - 1];
    }
    else {
        adcValue = adcBuffer[0];
    }

    // Konwertujemy 12-bitową wartość ADC (0-4095) na 2 znaki ASCII
    char firstChar = (adcValue >> 6) & 0x3F;  // Bierzemy górne 6 bitów
    char secondChar = adcValue & 0x3F;        // Bierzemy dolne 6 bitów

    // Konwertujemy wartości 0-63 na znaki ASCII (64 możliwe wartości)
    firstChar += 0x20;   // 0x20 to spacja w ASCII, daje nam zakres od spacji do '?'
    secondChar += 0x20;

    // Tworzymy odpowiedź z 2 zakodowanymi znakami
    uint8_t responseLen = snprintf(response, sizeof(response), "\r\nSensor level - [%c%c]\r\n", firstChar, secondChar);
    sendFrameResponse(response, responseLen, sender);
}

void handleShowStatus(const char* sender) {
    FATFS* fs;                       // Wskaźnik do struktury zawierającej informacje o systemie plików
    DWORD fre_clust, total_sectors, free_sectors;  // Zmienne 32-bitowe na dane o przestrzeni
    char responseBuffer[80];        // Bufor na sformatowaną odpowiedź
    uint16_t responseLen;            // Długość odpowiedzi

    // Montowanie systemu plików
    // Pusty string "" oznacza domyślny wolumin
    // 1 oznacza natychmiastowe montowanie
    // Jeśli wystąpi błąd, wysyła komunikat o błędzie i kończy działanie
    if(f_mount(&FatFs, "", 1) != FR_OK) {
        sendFrameResponse("\r\nError mounting filesystem\r\n", 28, sender);
        return;
    }

    // Pobieranie informacji o wolumenie (przestrzeni dyskowej)
    // f_getfree pobiera informacje o wolnej przestrzeni
    // Zapisuje liczbę wolnych klastrów do fre_clust
    // Aktualizuje wskaźnik fs informacjami o systemie plików
    if(f_getfree("", &fre_clust, &fs) != FR_OK) {
        sendFrameResponse("\r\nError getting volume info\r\n", 27, sender);
        f_mount(NULL, "", 0);  // Odmontowuje system plików przed wyjściem
        return;
    }

    // Obliczanie całkowitej i wolnej przestrzeni
    // n_fatent - 2 to liczba dostępnych klastrów (odejmujemy 2 zarezerwowane klastry)
    // csize to liczba sektorów na klaster
    total_sectors = (fs->n_fatent - 2) * fs->csize;
    free_sectors = fre_clust * fs->csize;

    // Formatowanie odpowiedzi
    // Dzieli wartości przez 2 aby przekonwertować sektory (512B) na KB
    responseLen = snprintf(responseBuffer, sizeof(responseBuffer),
        "\r\nSD Card Status\r\n"
        "Total space - %lu KB\r\n"
        "Free space - %lu KB\r\n",
        total_sectors / 2,
        free_sectors / 2);

    // Wysyłanie odpowiedzi
    sendFrameResponse(responseBuffer, responseLen, sender);

    // Odmontowanie systemu plików
    f_mount(NULL, "", 0);
}

void handleListCommand(const char* data, uint16_t dataLen, const char* sender) {
    DIR dir;                          // Struktura katalogu
    FILINFO fno;                      // Informacje o pliku/katalogu
    char responseBuffer[256];         // Bufor na odpowiedź
    uint16_t responseLen = 0;         // Długość odpowiedzi
    const char* path;                 // Ścieżka do katalogu
    const char* header = "\r\nDirectory listing -";
    const uint16_t maxDataLength = 256; // Maksymalna długość danych w ramce

    // Sprawdza czy komenda zawiera ścieżkę (np. "ls /folder")
    if(dataLen > 3 && memcmp(data, "ls ", 3) == 0) {
        path = data + 3;              // Jeśli tak, używa podanej ścieżki
    } else {
        path = "/";                   // Jeśli nie, używa katalogu głównego
    }

    // Montuje system plików
    if(f_mount(&FatFs, "", 1) != FR_OK) {
        sendFrameResponse("\r\nError mounting filesystem\r\n", 28, sender);
        return;
    }

    // Otwiera katalog o podanej ścieżce
    if(f_opendir(&dir, path) != FR_OK) {
        sendFrameResponse("\r\nError opening directory\r\n", 27, sender);
        f_mount(NULL, "", 0);
        return;
    }

    // Inicjalizuje pierwszy bufor odpowiedzi z nagłówkiem i ścieżką
    responseLen = snprintf(responseBuffer, sizeof(responseBuffer),
                          "%s %s", header, path);

    // Czyta zawartość katalogu
    while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
        // Pomija "." i ".."
        if(fno.fname[0] == '.') continue;

        // Oblicza długość następnej linii
        uint16_t lineLen = 2;  // poniewaz \r\n
        lineLen += strlen(fno.fname);
        if(fno.fattrib & AM_DIR) lineLen++; // '#' dla katalogów

        // Jeśli następna linia przekroczy limit, wyślij aktualny bufor i rozpocznij nowy
        if(responseLen + lineLen > maxDataLength) {
            // Wysyła aktualny bufor
            sendFrameResponse(responseBuffer, responseLen, sender);

            // Rozpoczyna nowy bufor z nagłówkiem
            responseLen = snprintf(responseBuffer, sizeof(responseBuffer),
                                 "%s %s", header, path);
        }

        //poczatek dokumentu + ile juz napisalismy

        // Dodaje nową linię
        responseLen += snprintf(responseBuffer + responseLen,  // to miejsce gdzie postawimy kursor aby pisać dalej
                              sizeof(responseBuffer) - responseLen, // ile miejsca zostało w buforze
                              "\r\n%s%s",
                              fno.fname,
                              (fno.fattrib & AM_DIR) ? "#" : "");
    }

    // Wysyła ostatni bufor jeśli zawiera dane
    if(responseLen > 0) {
        sendFrameResponse(responseBuffer, responseLen, sender);
    }

    // Zamyka katalog i odmontowuje system plików
    f_closedir(&dir);
    f_mount(NULL, "", 0);
}

void handleMkdirCommand(const char* data, uint16_t dataLen, const char* sender) {
    // Pomija prefiks "mkdir " (6 znaków)
    const char* dirName = data + 6;      // Przesuwa wskaźnik za "mkdir "
    uint16_t dirNameLen = dataLen - 6;   // Oblicza długość nazwy katalogu

    // Sprawdza czy podano nazwę katalogu
    if(dirNameLen == 0) {
        sendFrameResponse("\r\nError directory name not provided\r\n", 36, sender);
        return;
    }

    // Tworzy tymczasowy bufor na nazwę (z terminatorem null)
    char tempName[256];
    memcpy(tempName, dirName, dirNameLen);
    tempName[dirNameLen] = '\0';

    // Sprawdza poprawność nazwy
    if(!isValidName(tempName)) {
        sendFrameResponse("\r\nError: Invalid directory name\r\n", 31, sender);
        return;
    }

    // Montuje system plików
    if(f_mount(&FatFs, "", 1) != FR_OK) {
        sendFrameResponse("\r\nError mounting filesystem\r\n", 28, sender);
        return;
    }

    // Tworzy katalog
    FRESULT res = f_mkdir(dirName);

    // Obsługuje wynik operacji
    if(res == FR_OK) {
        sendFrameResponse("\r\nThe directory has been created\r\n", 33, sender);
    } else {
        sendFrameResponse("\r\nError creating directory\r\n", 27, sender);
    }

    // Odmontowuje system plików
    f_mount(NULL, "", 0);
}

void handleRemoveCommand(const char* data, uint16_t dataLen, const char* sender) {
    // Pomija prefiks "rm " (3 znaki)
    const char* path = data + 3;         // Przesuwa wskaźnik za "rm "
    uint16_t pathLen = dataLen - 3;      // Oblicza długość ścieżki

    // Sprawdza czy podano ścieżkę
    if(pathLen == 0) {
        sendFrameResponse("\r\nError: Path not provided\r\n", 28, sender);
        return;
    }

    // Montuje system plików
    if(f_mount(&FatFs, "", 1) != FR_OK) {
        sendFrameResponse("\r\nError mounting filesystem\r\n", 28, sender);
        return;
    }

    // Usuwa plik lub katalog
    FRESULT res = f_unlink(path);

    // Obsługuje wynik operacji, gdy w katalogu cos bedzie operacja tez sie nie powiedzie
    if(res == FR_OK) {
        sendFrameResponse("\r\nThe file or directory has been deleted\r\n", 41, sender);
    } else {
        sendFrameResponse("\r\nError deleting file or directory\r\n", 35, sender);
    }

    // Odmontowuje system plików
    f_mount(NULL, "", 0);
}

void handleMkfileCommand(const char* data, uint16_t dataLen, const char* sender) {
    // Pomija prefiks "mkfile " (7 znaków)
    const char* fileName = data + 7;      // Przesuwa wskaźnik za "mkfile "
    uint16_t fileNameLen = dataLen - 7;   // Oblicza długość nazwy pliku

    // Sprawdza czy podano nazwę pliku
    if(fileNameLen == 0) {
        sendFrameResponse("\r\nError: File name not provided\r\n", 33, sender);
        return;
    }

    // Tworzy tymczasowy bufor na nazwę (z terminatorem null)
    char tempName[256];
    memcpy(tempName, fileName, fileNameLen);
    tempName[fileNameLen] = '\0';

    // Sprawdza poprawność nazwy
    if(!isValidName(tempName)) {
        sendFrameResponse("\r\nError: Invalid file name\r\n", 28, sender);
        return;
    }

    // Montuje system plików
    if(f_mount(&FatFs, "", 1) != FR_OK) {
        sendFrameResponse("\r\nError mounting filesystem\r\n", 28, sender);
        return;
    }

    // FA_CREATE_ALWAYS - zawsze tworzy nowy plik (jeśli istnieje, zostanie nadpisany)
    // FA_WRITE - otwiera plik w trybie zapisu

    // Tworzy plik
    FIL file;
    FRESULT res = f_open(&file, fileName, FA_CREATE_ALWAYS | FA_WRITE);
    if(res == FR_OK) {
        f_close(&file);
        sendFrameResponse("\r\nThe file has been created\r\n", 29, sender);
    } else {
        sendFrameResponse("\r\nError creating file\r\n", 23, sender);
    }

    // Odmontowuje system plików
    f_mount(NULL, "", 0);
}

void handleReadCommand(const char* data, uint16_t dataLen, const char* sender) {
    // Pomija prefiks "read " (5 znaków)
    const char* fileName = data + 5;      // Przesuwa wskaźnik za "read "
    uint16_t fileNameLen = dataLen - 5;   // Oblicza długość nazwy pliku

    // Sprawdza czy podano nazwę pliku
    if(fileNameLen == 0) {
        sendFrameResponse("\r\nError: File name not provided\r\n", 33, sender);
        return;
    }

    // Montuje system plików
    if(f_mount(&FatFs, "", 1) != FR_OK) {
        sendFrameResponse("\r\nError mounting filesystem\r\n", 28, sender);
        return;
    }

    // Otwiera plik w trybie odczytu
    FIL file;
    FRESULT res = f_open(&file, fileName, FA_READ);
    if(res != FR_OK) {
        sendFrameResponse("\r\nError opening file\r\n", 22, sender);
        f_mount(NULL, "", 0);
        return;
    }

    // Pobiera rozmiar pliku
    FSIZE_t fileSize = f_size(&file);

    // Maksymalny rozmiar danych w jednej ramce (256 bajtów - maksymalna długość pola danych)
    const uint16_t maxDataPerFrame = 256;

    // Bufor na zawartość pliku
    char buffer[maxDataPerFrame];
    UINT bytesRead;
    FSIZE_t totalBytesRead = 0;

    // Czyta i wysyła plik w częściach
    while(totalBytesRead < fileSize) { // Pętla działa dopóki nie przeczytamy całego pliku
        // Sprawdza czy pozostała część pliku (fileSize - totalBytesRead) jest większa niż 256 bajtów (maxDataPerFrame)
    	// Jeśli TAK: czyta 256 bajtów
    	// Jeśli NIE: czyta tylko pozostałą część pliku
        UINT bytesToRead = (fileSize - totalBytesRead > maxDataPerFrame) ?
                          maxDataPerFrame : (UINT)(fileSize - totalBytesRead); // rzutowanie bo sa fsize

        // Czyta fragment pliku
        res = f_read(&file, buffer, bytesToRead, &bytesRead);
        if(res != FR_OK || bytesRead == 0) {
            sendFrameResponse("\r\nError reading file\r\n", 22, sender);
            break;
        }

        // Wysyła fragment pliku
        sendFrameResponse(buffer, bytesRead, sender);

        // Aktualizuje licznik przeczytanych bajtów
        totalBytesRead += bytesRead;
    }

    // Zamyka plik
    f_close(&file);

    // Odmontowuje system plików
    f_mount(NULL, "", 0);
}

void handleWriteCommand(const char* data, uint16_t dataLen, const char* sender) {
    // Pomija prefiks "write " (6 znaków)
    const char* fileName = data + 6;      // Przesuwa wskaźnik za "write "
    uint16_t fileNameLen = dataLen - 6;   // Oblicza długość nazwy pliku

    // Sprawdza czy podano nazwę pliku
    if(fileNameLen == 0) {
        sendFrameResponse("\r\nError: File name not provided\r\n", 33, sender);
        return;
    }

    // Sprawdza czy inny plik nie jest już otwarty do zapisu
    if(fileOpenForWrite) {
        sendFrameResponse("\r\nError: Another file is already open for writing\r\n", 49, sender);
        return;
    }

    // Montuje system plików
    if(f_mount(&FatFs, "", 1) != FR_OK) {
        sendFrameResponse("\r\nError mounting filesystem\r\n", 28, sender);
        return;
    }

    // Otwiera plik do zapisu
    FRESULT res = f_open(&writeFile, fileName, FA_WRITE | FA_CREATE_ALWAYS);
    if(res != FR_OK) {
        sendFrameResponse("\r\nError opening file for writing\r\n", 34, sender);
        f_mount(NULL, "", 0);
        return;
    }

    // Ustawia flagę wskazującą, że plik jest otwarty do zapisu
    fileOpenForWrite = 1;

    // Wysyła odpowiedź że jest gotowy na dane
    sendFrameResponse("\r\nFile is opened. Send data frame:\r\n", 35, sender);
}

void handleSendCommand(const char* data, uint16_t dataLen, const char* sender) {
    // Pomija prefiks "send " (5 znaków)
    const char* writeData = data + 5;      // Przesuwa wskaźnik za "send "
    uint16_t writeDataLen = dataLen - 5;   // Oblicza długość danych do zapisu

    // Sprawdza czy jakiś plik jest otwarty do zapisu
    if(!fileOpenForWrite) {
        sendFrameResponse("\r\nError: No file open for writing\r\n", 35, sender);
        return;
    }

    // Zapisuje dane do pliku
    UINT bytesWritten;
    FRESULT res = f_write(&writeFile, writeData, writeDataLen, &bytesWritten);

    // Zamyka plik
    f_close(&writeFile);
    fileOpenForWrite = 0;  // Zeruje flagę

    // Odmontowuje system plików
    f_mount(NULL, "", 0);

    // Sprawdza czy zapis się powiódł
    if(res != FR_OK || bytesWritten != writeDataLen) {
        sendFrameResponse("\r\nError writing to file\r\n", 24, sender);
    } else {
        sendFrameResponse("\r\nFile written successfully\r\n", 28, sender);
    }
}

// Funkcja sprawdzająca poprawność nazwy pliku/katalogu
uint8_t isValidName(const char* name) {
    // Sprawdzenie niedozwolonych znaków
    const char* invalidChars = "<>:\"/\\|?*";
    for(uint16_t i = 0; name[i] != '\0'; i++) {
        if(strchr(invalidChars, name[i]) != NULL) {
            return 0;
        }
    }
    return 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  // Rozpoczęcie odbierania danych
  HAL_UART_Receive_IT(&huart2, &usartReceiveBuffer[0], 1);

  HAL_TIM_Base_Start(&htim2);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);

  // Bufor na odebrane dane
  char inputBuffer[FRAME_MAX_LENGTH];
  uint16_t receivedLength;


  sendFrameResponse("\r\nSystem Started\r\n", 18, TERMINAL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Jeśli odebrano kompletną linię
	  if((receivedLength = readUsartLine(inputBuffer)) > 0) {
		  processReceivedLine(inputBuffer, receivedLength);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
