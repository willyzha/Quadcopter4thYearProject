void setup() 
{
  Serial.begin(9600);
}

// serial command buffer
char buf[255];
char out[255];
int buf_offset = 0;

void loop() 
{
  int totalBytes = Serial.available();
  if(totalBytes > 0) // check to see if data is present
  {
    while(totalBytes > 0) // start loop with number of bytes
    {
      char c = (char)Serial.read(); // read next byte
      if(c == '\n')
      {
        buf[buf_offset] = '\0'; // null terminator
        // process data
        //Serial.println(buf);
        char *str = strtok(buf," ,"); //str = roll,pitch,throttle,yaw
        strcat(out,str);
        while (str != NULL) // loop to print out each token 
        {
          //Serial.println(str);
          str = strtok(NULL," ,");  
          strcat(out,str);  
        }
        Serial.println(out);
        buf_offset = 0;
      }
      else // when newline is reached
      {
        buf[buf_offset++] = c; // store in buffer and continue until newline is found
      }
      totalBytes--;
    }
  }
  
  

}
