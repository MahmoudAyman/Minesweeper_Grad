int r = 8;
int c = 9;
int l = 10;

char state='n'; 
char laststate='n';

void setup()
{
  pinMode(r, INPUT);
  pinMode(c, INPUT);
  pinMode(l, INPUT);
  
  Serial.begin(9600);
}

void loop()
{
  int right = digitalRead(r);
  int center = digitalRead(c);
  int left = digitalRead(l);
//  
//  Serial.print("Right: ");
//  Serial.print(right);
//  Serial.print(" , ");
//  
//  Serial.print("Center: ");
//  Serial.print(center);
//  Serial.print(" , ");
//  
//  Serial.print("Left: ");
//  Serial.print(left);
//  Serial.println(" , ");
//  
//  delay(500);
  
  if((center==0) && (left==1) && (right==1)){
    state='f';
  }
  else if ((center==0) && (left==0) && (right==1)){
    state='e';
  }
  else if ((center==0) && (left==0) && (right==1)){
    state='e';
  }

  else if ((center==1) && (left==1) && (right==0)){
    state='r';
  }
  else if ((center==1) && (left==0) && (right==1)){
    state='l';
  }
  else if ((center==0) && (left==0) && (right==0)){
    state='i';
  }
  
  
  Serial.print("state: ");
  Serial.print(state);
  Serial.print(" , ");
  Serial.print("laststate: ");
  Serial.print(laststate);
  Serial.print(" , ");
  
  
  if((state=='r') && (laststate=='i')){
    Serial.println("Go right");
  }
  
  else if((state=='l') && (laststate=='i')){
    Serial.println("Go left");
  }
  else if((state=='l') && (laststate=='l')){
    Serial.println("Go left");
  }
  else if((state=='f') && (laststate=='i')){
    Serial.println("Go forward");
  }
  else if((state=='f') && (laststate=='f')){
    Serial.println("Continue");
  }
  else if((state=='i') && (laststate=='f')){
    Serial.println("Wait for decision");
  }
  else if((state=='l') && (laststate=='f')){
    Serial.println("go left");
  }
  else if((state=='r') && (laststate=='f')){
    Serial.println("go right");
  }
  else if((state=='r') && (laststate=='r')){
    Serial.println("go right");
  }
  else if((state=='e') && (laststate=='f')){
    Serial.println("Wait for decision");
  }
  else if((state=='i') && (laststate=='i')){
    Serial.println("Wait for decision");
  }
  else{
    Serial.println("state not defined");
  }
 
  laststate=state;
  delay(200);
}
  
