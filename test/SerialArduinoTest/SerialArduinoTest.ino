int pi_rc_1;
int pi_rc_2;
int pi_rc_3;
int pi_rc_4;

void setup() 
{
  Serial.begin(115200);
}

// serial command buffer
char buf[255];
char out[255];

int chs;
int compareSum;
int val[4];
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

static void send_data(char* info)
{
  char sent[255];
  int sum = checkSum(info);
  char sumBuffer[5]; 
  sent[0] = '\0';
  itoa(sum,sumBuffer,10);
  strcat(sent,sumBuffer);
  strcat(sent,"*Flag: ");
  strcat(sent,info);
  Serial.println(sent);  
}

static void update_channel(int a, int b, int c, int d)
{
  if (a == 9999)
  {
    pi_rc_2 = b;
    pi_rc_3 = c;
    pi_rc_4 = d;
  }
  else if (b == 9999)
  {
    pi_rc_1 = a;
    pi_rc_3 = c;
    pi_rc_4 = d;          
  }
  else if (c == 9999)
  {
    pi_rc_1 = a;
    pi_rc_2 = b;
    pi_rc_4 = d;
  }
  else if (d == 9999)
  {
    pi_rc_1 = a;
    pi_rc_2 = b;
    pi_rc_3 = c;  
  }
  else
  {
    pi_rc_1 = a;
    pi_rc_2 = b;
    pi_rc_3 = c;
    pi_rc_4 = d;
  }
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
      if(c == '\n') // when \n is reached
      {
        out[0] = '\0'; //reset out char arrays
        buf[buf_offset] = '\0'; // null terminator
        // process data
        char *chk = strtok(buf," *,"); //obtain checkSum
        char *str = strtok(NULL," *,"); //str = roll,pitch,throttle,yaw
        
        while (str != NULL) // loop to go through each token
        {
          strcat(out,str);
          strcat(out," ");
          val[counter++] = strtol(str,NULL,16); //saving values of each token as long
          str = strtok(NULL," *,");
        }
        
        out[strlen(out)-1] = '\0';
       
        //calculate checksum and convert char chk into int
        chs = checkSum(out);
        compareSum = strtol(chk,NULL,10);
        //compare checksum value with value from python
        if (chs == compareSum)
        {
          send_data(out);
          //set channel values using val[] from while loop
          update_channel(val[0], val[1], val[2], val[3]);
        }
        else
        {
          send_data("CheckSum Failed");
        }
        buf_offset = 0; //reset buf_offset
      }
      else //c is not \n
      {
        buf[buf_offset++] = c; // store in buffer and continue until newline is found
      }
      
      totalBytes--;
    }
  }
}
