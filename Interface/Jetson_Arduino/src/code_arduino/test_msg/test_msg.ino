#define SIZE_BUFFER 10


size_t bytes_received;
char identificador = ',';
char msg_buffer[SIZE_BUFFER+1];
char msg_decode[SIZE_BUFFER];
int msg_decode_int[8];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  bytes_received = Serial.readBytesUntil('\n', msg_buffer, SIZE_BUFFER);
  if (bytes_received > 0){
      
    for(int i = 0, j = 0; i<SIZE_BUFFER+1; i++){
      if(msg_buffer[i] != ','){
        msg_decode[i] = msg_buffer[i];
      }else{
        msg_decode_int[i] = atoi(msg_decode);
        Serial.println(msg_decode_int[j++]);
      }
    }
  }
}
