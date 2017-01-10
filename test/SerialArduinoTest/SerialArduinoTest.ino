void setup() 
{
  Serial.begin(115200);
}

// serial command buffer
char buf[255];
char out[255];
char full[255];
int chs;
int compareSum;
long val[4];
int counter;
int buf_offset = 0;

int checkSum(char *str)
{
  int sum = 0;
  int len = strlen(str);
  for(int i=0;i<len;i++)
  {
    sum += str[i];
  }
  return sum;
}

void loop() 
{
  int timer = millis();
  int totalBytes = Serial.available();
  if(totalBytes > 0) // check to see if data is present
  {
    counter = 0;
    while(totalBytes > 0) // start loop with number of bytes
    {
      char c = (char)Serial.read(); // read next byte    
      if(c == '\n')
      {
        buf[buf_offset] = '\0'; // null terminator
        // process data
        char *chk = strtok(buf," *,"); //obtain checkSum
        char *str = strtok(NULL," *,"); //str = roll,pitch,throttle,yaw
        strcat(out,"Flag: ");
        
        while (str != NULL) // loop to go through each token
        {
          strcat(out,str);
          strcat(full,str);
          strcat(out,", ");
          strcat(full,", ");
          val[counter++] = strtol(str,NULL,16); //saving values of each token as long
          str = strtok(NULL," ,");
        }
        
        out[strlen(out)-2] = "\0";
        full[strlen(full)-2] = "\0";

        //calculate checksum and convert char chk into int
        chs = checkSum(full);
        sscanf(chk,"%d",&compareSum);
        //compare checksum value with value from python
        if (chs == compareSum)
        {
          Serial.println(out);
          //set channel values using val[] from while loop
        }
        else
        {
          Serial.println("Flag: CheckSum Fail");
        }
        buf_offset = 0; //reset buf_offset
      }
      else // when newline is reached
      {
        buf[buf_offset++] = c; // store in buffer and continue until newline is found
      }
      
      totalBytes--;
      //reset out and full char arrays
      out[0] = "\0";
      full[0] = "\0";
    }
  }
}
