// ------- prelude for SD card section
# include <SD.h> // library for writing and reading from the SD card

// don't need these definitions, just useful to make sure they're 
//claimed as SD card stuff and not forgotten about later
const int MOSI_reminder = 11; // to DI
const int MISO_reminder = 12; // to DO
const int CLK_reminder = 13; // to CLK

// chip select pin can be anything
const int CS = 4; 

// minimum for an 8.3 filename
const int filename_bytes = 12;

// useful to have it as a global variable
char* filename;

Sd2Card card;

void setup() {
  Serial.begin(9600);
  Serial.println("serial is working");
  
  // initialise the card on the defined pin
  initCard(CS);
}

void loop() {
  // test the function every now and again
  write_to_card(filename, "definitely working");
  delay(1000);
}

void write_to_card(char* file, char* toWrite) {
  /*
   * natalie's responsibility
   * 
   * takes in a filename and writes to it, and closes the file afterwards 
   */
  Serial.print("writing to ");
  Serial.println(file);
  int now = millis();
  File location = SD.open(file, FILE_WRITE);
  location.print(now);
  location.print(": ");
  location.println(toWrite);
  location.close();
}

bool initCard(int pin) {
  /*
   * natalie's responsibility
   * 
   * tries to intialise the SD card and get the filename to use
   */
  
  SD.begin(pin);
  
  // can the card be initialised?
  if (!card.init(SPI_HALF_SPEED, CS)) {
    return false;
  }

  filename = get_filename();
  
  return true;
}

char* get_filename() {
  /*
   * function to get the filename to use, so each test gets a unique 
   * file
   * 
   */
  int i = 0;

  // files will follow the pattern test(number).csv
  char base_name[] = "test";
  char ending[] = ".csv";

  // need to allocate so the filename can be stored.
  // won't free this variable as it should always be useful
  char* return_memory = malloc(filename_bytes * sizeof(char));
  
  while (true) {
    // this checks every loop to see if a filename is free. 
    // if so, it returns it
    // if not, it tries the next one
    
    char filename [filename_bytes];
    sprintf(filename, "%s%d%s", base_name, i, ending); // formats the filename
    if (SD.exists(filename)) {
      i++;
    }else{
      strcpy(return_memory, filename);
      return return_memory;
    }
  }
}
