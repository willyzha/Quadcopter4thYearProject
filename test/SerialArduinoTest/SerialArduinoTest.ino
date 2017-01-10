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
    //Serial.print(str[i],DEC);
    //Serial.print(" ");
    //Serial.println(sum,DEC);
  }
  //Serial.println(sum);
  
  return sum;
}

void loop() 
{
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
        //Serial.println(buf);
        char *chk = strtok(buf," *,"); //obtain checkSum
        char *str = strtok(NULL," *,"); //str = roll,pitch,throttle,yaw
        strcat(out,"Flag: ");
        while (str != NULL) // loop to print out each token 
        {
          //Serial.println(str);
          strcat(out,str);
          strcat(full,str);
          strcat(out,", ");
          strcat(full,", ");
          val[counter] = strtol(str,NULL,16);
          str = strtok(NULL," ,");  
        }
        out[strlen(out)-2] = "\0";
        full[strlen(full)-2] = "\0";
        chs = checkSum(full);
        //Serial.println(chk);
        Serial.println(chs);
        sscanf(chk,"%d",&compareSum);
        Serial.println(compareSum);
        if (chs == compareSum)
        {
          Serial.println("Success");
          Serial.println(out);
        }
        else
        {
          Serial.println("Fail");
          Serial.println(full);
        }
        buf_offset = 0;
      }
      else // when newline is reached
      {
        buf[buf_offset++] = c; // store in buffer and continue until newline is found
      }
      
      totalBytes--;
      out[0] = "\0";
      full[0] = "\0";
    }
   // Serial.println("Garbage");
  }
}
