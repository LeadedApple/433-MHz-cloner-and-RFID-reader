// Dołączenie potrzebnych bibliotek
#include <Wire.h>            // Komunikacja I2C
#include <LiquidCrystal_I2C.h> // Sterowanie wyświetlaczem LCD przez I2C
#include <RCSwitch.h>        // Obsługa modułu RF 433 MHz
#include <SPI.h>             // Komunikacja SPI
#include <MFRC522.h>         // Obsługa czytnika kart RFID

// Deklaracja pinów używanych w projekcie
int rfReceiverPin = 2;       // Pin odbiornika RF
int rfTransmitterPin = 4;    // Pin nadajnika RF
int I2C_SDA = 21;            // Pin danych I2C
int I2C_SCL = 22;            // Pin zegara I2C
int joyx = 33;               // Pin odczytu osi X joysticka
int joyy = 32;               // Pin odczytu osi Y joysticka
int rst_pin = 25;            // Pin resetu dla RFID
int ss_pin = 13;             // Pin SS dla RFID
int ledr = 12;               // Pin czerwonej diody LED
int ledg = 14;               // Pin zielonej diody LED
int ledb = 26;               // Pin niebieskiej diody LED

// Zmienne pomocnicze do obsługi czasu i przechowywania danych o sygnałach RF
unsigned long currentSeconds, previousSecondsRecord;
unsigned long receivedCodes[100]; // Bufor przechowujący odebrane kody RF
int receivedProtocols[100];       // Bufor przechowujący protokoły odebranych kodów
int receiverCounter = 0;          // Licznik odebranych sygnałów
bool canStoreCurrentCode = true;  // Flag kontrolująca możliwość zapisu kodu
unsigned long previousMillisLed = 0; // Poprzedni czas zmiany stanu LED
bool blinkLed = false;            // Flag kontrolująca miganie LED
int ledCounter = 0;               // Licznik migania LED
unsigned long messageStartTime = 0;  // Czas rozpoczęcia wyświetlania komunikatu
const unsigned long messageDuration = 5000; // Czas trwania wyświetlanego komunikatu
bool messageActive = false;       // Flag kontrolująca wyświetlanie komunikatu
unsigned long lastReceivedCode = 0; // Ostatni odebrany kod RF
int lastReceivedProtocol = 0;     // Protokół ostatniego odebranego kodu

// Inicjalizacja obiektów dla urządzeń
MFRC522 mfrc522(ss_pin, rst_pin);  // Obiekt czytnika RFID
RCSwitch mySwitch = RCSwitch();    // Obiekt przełącznika RF
LiquidCrystal_I2C lcd(0x27, 16, 2); // Obiekt LCD (adres I2C, liczba kolumn, liczba wierszy)

// Typ wyliczeniowy dla opcji menu
enum MenuOption {
  OPTION_433MHZ,             // Opcja dla modułu RF 433 MHz
  OPTION_RFID,               // Opcja dla czytnika RFID
  SUBOPTION_433MHZ_CAPTURE,  // Podopcja dla przechwytywania sygnału RF
  SUBOPTION_433MHZ_SEND,     // Podopcja dla wysyłania sygnału RF
  SUBOPTION_RFID_READ,       // Podopcja dla odczytu tagu RFID
  SUBOPTION_RFID_WRITE       // Podopcja dla zapisu tagu RFID
};

MenuOption currentOption = OPTION_433MHZ; // Aktualnie wybrana opcja w menu

// Funkcja setup() wywoływana raz przy starcie programu
void setup() {
  pinMode(ledr, OUTPUT);
  pinMode(ledg, OUTPUT);
  pinMode(ledb, OUTPUT);
  setLedColor(255, 255, 255); // Ustawienie koloru LED na biały

  SPI.begin(16, 19, 27, 13);  // Inicjalizacja SPI
  mfrc522.PCD_Init();         // Inicjalizacja czytnika RFID
  Serial.begin(115200);       // Rozpoczęcie komunikacji szeregowej z prędkością 115200 bps
  mySwitch.enableReceive(rfReceiverPin);  // Włączenie odbiornika RF
  mySwitch.enableTransmit(rfTransmitterPin);  // Włączenie nadajnika RF
  Serial.print("Initializing SPI bus and RFID reader...");
  mfrc522.PCD_DumpVersionToSerial();  // Wypisanie wersji czytnika RFID na konsolę
  lcd.clear();                // Czyszczenie wyświetlacza LCD
  lcd.init();                 // Inicjalizacja LCD
  lcd.backlight();            // Włączenie podświetlenia LCD
  Serial.println("Setup completed"); // Komunikat o zakończeniu konfiguracji
}

// Funkcja loop() wywoływana w pętli po setup()
void loop() {
  currentSeconds = millis() / 1000;  // Aktualny czas w sekundach
  decodeRfSignals();           // Funkcja dekodująca sygnały RF
  checkLedState();             // Sprawdzenie i aktualizacja stanu LED
  handleJoystick();            // Obsługa joysticka

  // Sprawdzenie, czy na czytniku RFID pojawiła się nowa karta
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.println("Card detected"); // Jeśli tak, wypisanie komunikatu
  }

  // Obsługa wygasania komunikatu na wyświetlaczu po określonym czasie
  if (messageActive && (millis() - messageStartTime >= messageDuration)) {
    setLedColor(255, 0, 0);  // Zmiana koloru LED na czerwony
    lcd.clear();             // Czyszczenie wyświetlacza
    lcd.setCursor(0, 0);
    lcd.print("> Przechwyc sygnal");  // Wyświetlenie tekstu
    lcd.setCursor(0, 1);
    lcd.print("Czekam na sygnal");
    messageActive = false;   // Deaktywacja flagi wyświetlania komunikatu
  }

  // Obsługa wysyłania ostatnio odebranego kodu RF
  if (currentOption == SUBOPTION_433MHZ_SEND) {
    transmitLastReceivedCode(); // Funkcja nadająca sygnał RF
  }

  // Obsługa odczytu tagu RFID
  if (currentOption == SUBOPTION_RFID_READ) {
    readRfidTag();  // Funkcja odczytująca tag RFID
  }

  displayMenuOptions();  // Funkcja wyświetlająca opcje menu
}

// Funkcja dekodująca odebrane sygnały RF
void decodeRfSignals() {
  if (mySwitch.available()) { // Sprawdzenie, czy dostępny jest jakiś sygnał
    setLedColor(0, 255, 0);   // Zmiana koloru LED na zielony
    Serial.println("Signal received");  // Komunikat o odebraniu sygnału
    blinkLed = true;           // Włączenie migania LED

    unsigned long receivedValue = mySwitch.getReceivedValue(); // Odczytanie wartości sygnału
    int receivedProtocol = mySwitch.getReceivedProtocol(); // Odczytanie protokołu sygnału

    // Obsługa opcji przechwytywania sygnałów RF
    if (currentOption == SUBOPTION_433MHZ_CAPTURE) {
      receivedCodes[receiverCounter] = receivedValue; // Zapisanie wartości sygnału do bufora
      receivedProtocols[receiverCounter] = receivedProtocol; // Zapisanie protokołu do bufora

      lastReceivedCode = receivedValue; // Zapisanie ostatnio odebranego kodu
      lastReceivedProtocol = receivedProtocol; // Zapisanie protokołu ostatniego sygnału

      lcd.clear();              // Czyszczenie LCD
      lcd.setCursor(0, 0);
      lcd.print("Odebrano sygnal");  // Wyświetlenie informacji o odebranym sygnale
      lcd.setCursor(0, 1);
      lcd.print("Code: ");
      lcd.print(receivedValue);  // Wyświetlenie wartości sygnału

      messageStartTime = millis();  // Rozpoczęcie odmierzania czasu wyświetlania komunikatu
      messageActive = true;         // Aktywacja flagi wyświetlania komunikatu

      receiverCounter++;            // Inkrementacja licznika odebranych sygnałów
      if (receiverCounter >= 100) { // Jeśli bufor jest pełen, reset licznika
        receiverCounter = 0;
      }
    }

    mySwitch.resetAvailable();  // Resetowanie flagi dostępności sygnału
  }
}

// Funkcja nadająca ostatnio odebrany kod RF
void transmitLastReceivedCode() {
  if (lastReceivedCode != 0) { // Sprawdzenie, czy jakiś kod został odebrany
    mySwitch.setProtocol(lastReceivedProtocol);  // Ustawienie odpowiedniego protokołu
    mySwitch.send(lastReceivedCode, 24);         // Nadanie kodu
    lcd.clear();              // Czyszczenie wyświetlacza
    lcd.setCursor(0, 0);
    lcd.print("Nadawanie sygnalu:"); // Wyświetlenie komunikatu o nadawaniu
    lcd.setCursor(0, 1);
    lcd.print(lastReceivedCode);     // Wyświetlenie nadawanego kodu
    delay(1000); // Opóźnienie przed kontynuacją programu
  }
}

// Funkcja odczytująca tag RFID
void readRfidTag() {
    setLedColor(255, 255, 0);  // Zmiana koloru LED na żółty
    lcd.clear();               // Czyszczenie wyświetlacza
    lcd.setCursor(0, 0);
    lcd.print("> Zczytaj tag");  // Wyświetlenie komunikatu o odczycie tagu

    // Sprawdzenie, czy karta jest obecna i czy udało się ją odczytać
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
        lcd.setCursor(0, 1);
        lcd.print("Czekam na tag");  // Jeśli nie, wyświetlenie oczekiwania na kartę
        Serial.println("No card detected."); // Wypisanie informacji na konsolę
        return;  // Wyjście z funkcji
    }

    setLedColor(255, 20, 147);  // Zmiana koloru LED na różowy
    Serial.println("Card detected, reading UID");  // Komunikat o wykryciu karty
    lcd.setCursor(0, 1);
    lcd.print("UID:");  // Wyświetlenie etykiety UID

    String uidString = "";  // Zmienna na łańcuch reprezentujący UID
    for (byte i = 0; i < mfrc522.uid.size; i++) {
        char buf[4];  // Bufor na pojedynczy bajt UID
        sprintf(buf, "%02X", mfrc522.uid.uidByte[i]);  // Formatowanie bajtu jako szesnastkowego
        uidString += buf;  // Dodanie sformatowanego bajtu do łańcucha UID
        if (i < mfrc522.uid.size - 1) {
            uidString += " ";  // Dodanie spacji między bajtami
        }
        Serial.print(buf);  // Wypisanie bajtu na konsolę
        if (i < mfrc522.uid.size - 1) {
            Serial.print(" ");  // Wypisanie spacji na konsolę
        }
    }
    Serial.println();  // Nowa linia na konsoli

    lcd.setCursor(5, 1);  // Ustawienie kursora na wyświetlaczu
    lcd.print(uidString);  // Wyświetlenie UID
    delay(5000);  // Opóźnienie przed kontynuacją programu
    mfrc522.PICC_HaltA();  // Zatrzymanie komunikacji z kartą

    lcd.clear();  // Czyszczenie wyświetlacza
    lcd.setCursor(0, 0);
    lcd.print("> Zczytaj tag");  // Ponowne wyświetlenie komunikatu o odczycie
    lcd.setCursor(0, 1);
    lcd.print("Czekam na tag");  // Wyświetlenie oczekiwania na kartę
}

// Funkcja ustawiająca kolor LED
void setLedColor(int r, int g, int b) {
  analogWrite(ledr, r);  // Ustawienie intensywności czerwonego koloru
  analogWrite(ledg, g);  // Ustawienie intensywności zielonego koloru
  analogWrite(ledb, b);  // Ustawienie intensywności niebieskiego koloru
}

// Funkcja sprawdzająca i aktualizująca stan LED
void checkLedState() {
  if (blinkLed) {  // Jeśli flaga migania jest aktywna
    unsigned long currentMillisLed = millis();  // Pobranie aktualnego czasu
    if ((unsigned long)(currentMillisLed - previousMillisLed) >= 100) {  // Sprawdzenie, czy minęło 100 ms
      bool ledState = ledCounter % 2 == 0;  // Ustalenie stanu LED (zmiana co 100 ms)
      digitalWrite(ledr, ledState ? HIGH : LOW);  // Ustawienie stanu czerwonego LED
      digitalWrite(ledg, ledState ? HIGH : LOW);  // Ustawienie stanu zielonego LED
      digitalWrite(ledb, ledState ? HIGH : LOW);  // Ustawienie stanu niebieskiego LED
      Serial.print("LED State: "); Serial.println(ledState ? "HIGH" : "LOW");  // Wypisanie stanu LED na konsolę

      ledCounter++;  // Inkrementacja licznika LED
      if (ledCounter >= 4) {  // Jeśli licznik osiągnie wartość 4
        ledCounter = 0;       // Reset licznika
        blinkLed = false;     // Wyłączenie migania
      }
      previousMillisLed = currentMillisLed;  // Aktualizacja czasu ostatniej zmiany stanu LED
    }
  }
}


// Funkcja wyświetlająca opcje menu na LCD
void displayMenuOptions() {
  if (messageActive) return;  // Jeśli wyświetlany jest komunikat, nie aktualizuj menu

  static MenuOption lastOption = currentOption;  // Zmienna przechowująca ostatnio wybraną opcję
  if (lastOption != currentOption) {  // Jeśli aktualna opcja różni się od ostatniej
    lcd.clear();  // Wyczyść wyświetlacz
    lastOption = currentOption;  // Aktualizacja ostatnio wybranej opcji
  }

  lcd.setCursor(0, 0);  // Ustawienie kursora na początek pierwszego wiersza
  switch (currentOption) {  // Wybór działania w zależności od wybranej opcji
    case OPTION_433MHZ:
      setLedColor(255, 0, 0);  // Ustawienie koloru LED na czerwony
      lcd.print("> 433MHz");  // Wyświetlenie etykiety opcji RF
      lcd.setCursor(0, 1);
      lcd.print("  RFID");  // Wyświetlenie etykiety opcji RFID
      break;
    case OPTION_RFID:
      setLedColor(255, 255, 0);  // Ustawienie koloru LED na żółty
      lcd.print("  433MHz");  // Wyświetlenie etykiety opcji RF
      lcd.setCursor(0, 1);
      lcd.print("> RFID");  // Wyświetlenie etykiety opcji RFID
      break;
    case SUBOPTION_433MHZ_CAPTURE:
      lcd.print("> Przechwyc sygnal");  // Wyświetlenie etykiety podopcji przechwycenia sygnału RF
      lcd.setCursor(0, 1);
      lcd.print("Czekam na sygnal");  // Wyświetlenie informacji o oczekiwaniu na sygnał
      break;
    case SUBOPTION_433MHZ_SEND:
      lcd.print("  Przechwyc sygnal");  // Wyświetlenie etykiety podopcji przechwycenia sygnału RF
      lcd.setCursor(0, 1);
      lcd.print("> Nadaj sygnal");  // Wyświetlenie etykiety podopcji nadania sygnału RF
      break;
    case SUBOPTION_RFID_READ:
      lcd.print("> Zczytaj tag");  // Wyświetlenie etykiety podopcji odczytu tagu RFID
      lcd.setCursor(0, 1);
      lcd.print(mfrc522.PICC_IsNewCardPresent() ? "UID:" : "Czekam na tag");  // Wyświetlenie informacji o oczekiwaniu na kartę lub jej UID
      break;
    case SUBOPTION_RFID_WRITE:
      lcd.print("  Zczytaj tag");  // Wyświetlenie etykiety podopcji odczytu tagu RFID
      lcd.setCursor(0, 1);
      lcd.print("> Nadaj tag");  // Wyświetlenie etykiety podopcji zapisu na tag RFID
      break;
  }
}

// Funkcja obsługująca joystick
void handleJoystick() {
  int joyX = analogRead(joyx);  // Odczyt położenia joysticka w osi X
  int joyY = analogRead(joyy);  // Odczyt położenia joysticka w osi Y

  // Jeśli joystick jest przesunięty w górę
  if (joyY > 3200 + 500) {
    delay(200);  // Opóźnienie, aby zapobiec drganiom
    if (currentOption == OPTION_433MHZ || currentOption == OPTION_RFID) {
      enterSubMenu();  // Wejście do odpowiedniego submenu
    }
  } else if (joyY < 3200 - 500) {  // Jeśli joystick jest przesunięty w dół
    delay(200);  // Opóźnienie, aby zapobiec drganiom
    if (currentOption == SUBOPTION_433MHZ_CAPTURE || currentOption == SUBOPTION_433MHZ_SEND || currentOption == SUBOPTION_RFID_READ || currentOption == SUBOPTION_RFID_WRITE) {
      exitSubMenu();  // Wyjście z submenu
    }
  }

  // Przesunięcie joysticka w lewo lub w prawo zmienia aktualnie wybraną opcję
  if (joyX < 3200 - 500) {
    delay(200);  // Opóźnienie, aby zapobiec drganiom
    switchOption(false);  // Przejście do poprzedniej opcji
  } else if (joyX > 3200 + 500) {
    delay(200);  // Opóźnienie, aby zapobiec drganiom
    switchOption(true);  // Przejście do następnej opcji
  }
}

// Funkcja zmieniająca opcję menu
void switchOption(bool nextOption) {
  if (nextOption) {  // Jeśli wybrano przejście do następnej opcji
    // Cykliczne przemieszczanie się między opcjami
    if (currentOption == OPTION_433MHZ) {
      currentOption = OPTION_RFID;
    } else if (currentOption == OPTION_RFID) {
      currentOption = OPTION_433MHZ;
    } else if (currentOption == SUBOPTION_433MHZ_CAPTURE) {
      currentOption = SUBOPTION_433MHZ_SEND;
    } else if (currentOption == SUBOPTION_433MHZ_SEND) {
      currentOption = SUBOPTION_433MHZ_CAPTURE;
    } else if (currentOption == SUBOPTION_RFID_READ) {
      currentOption = SUBOPTION_RFID_WRITE;
    } else if (currentOption == SUBOPTION_RFID_WRITE) {
      currentOption = SUBOPTION_RFID_READ;
    }
  }
}

// Funkcja wchodząca do submenu
void enterSubMenu() {
  if (currentOption == OPTION_433MHZ) {
    currentOption = SUBOPTION_433MHZ_CAPTURE;  // Wejście do submenu przechwycenia sygnału RF
  } else if (currentOption == OPTION_RFID) {
    currentOption = SUBOPTION_RFID_READ;  // Wejście do submenu odczytu tagu RFID
  }
}

// Funkcja wychodząca z submenu
void exitSubMenu() {
  if (currentOption == SUBOPTION_433MHZ_CAPTURE || currentOption == SUBOPTION_433MHZ_SEND) {
    currentOption = OPTION_433MHZ;  // Powrót do głównej opcji RF
  } else if (currentOption == SUBOPTION_RFID_READ || currentOption == SUBOPTION_RFID_WRITE) {
    currentOption = OPTION_RFID;  // Powrót do głównej opcji RFID
  }
}
