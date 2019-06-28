import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

Serial port;

int serialCount=0;
int aligned=0;
boolean started=false;
boolean finished=false;
char [] packet= new char[12];
int [] mines = new int[40];
float [] path = new float[400];
boolean startMoving = false;
int points = 0;
int minesDetected = 0;
int interval = 0;

int deltaYaw;
float deltaX;
float deltaY;

int stepsF;
int stepsB;

boolean dir = true;
String msg;

int isMine = 0;
boolean detected =false;

long mineInterval;

float lastPosX = 0;
float lastPosY = 0;

int aRequest = -1;

Robot mineSweeper = new Robot(25, 25, 0);

void setup() {
  size(1000,1000);
  //lights();
  smooth();
    
  String portName = "/dev/ttyACM0";
  //printArray(Serial.list()[32]);
  
  port = new Serial(this, portName, 112500);
  //delay(1000);
  //port.buffer(7);
  //drawMap();
  
}
  
void draw() {
  if (millis() - interval > 1000) {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        port.write('r');
        interval = millis();
    }
    
  drawMap();
  
  //mineSweeper.move();
  stroke(120);
  strokeWeight(5);
  point(mineSweeper.getPosX(), mineSweeper.getPosY());
  strokeWeight(2);
  if((isMine == 1) && (!detected)){
    mines[minesDetected] = (int)mineSweeper.getPosX();
    minesDetected++;
    mines[minesDetected] = (int)mineSweeper.getPosY();
    minesDetected++;
    println(minesDetected);
    detected = true;
    mineInterval = millis();
  }
  else if(( isMine == 0) && ((millis() - mineInterval) > 2000)){
    detected = false;
    mineInterval = millis();
  }
  mineSweeper.rotateRobot(deltaYaw);
  mineSweeper.updatePosition();
  mineSweeper.drawRobot();
  //delay(500);
}


void drawMap()
{
  background(255);
  stroke(0);
  strokeWeight(2);
  int x=0;
  for (int i=0; i<20; i++)
  {
    line(x,0,x,height);
    x+=50;
  }
  int y=0;
  for (int i=0; i<20; i++)
  {
    line(0,y,width,y);
    y+=50;
  }
  
  for (int i=0; i<20; i=i+2){
    fill(255, 50, 0);
    if (!((mines[i] == 0) && (mines[i+1] == 0))){
      float locX = mines[i] /50;
      float locY = mines[i+1] /50;
      rect((locX*50)+12.5, (locY*50)+12.5, 25,25);
    }
    
  }
  
  for (int i=0; i<20; i=i+4){
    stroke(3);
    if (!((path[i] == 0) && (path[i+1] == 0))){
      line(path[i], path[i+1], path[i+2],path[i+3]);
    }
    
  }
  
}

class Robot{
  
  Robot(int h, int w, float angle)
  {
    c = color(200,30,0);
    posX=500;
    posY=500;
    height = h;
    width = w;
    orientation = angle;
    
  }
  
  //void move(float deltaX,float deltaY)
  //{
  //  posX+=deltaX;
  //  posY+=deltaY;
  //}
  
  void updatePosition(){
    float diff = 0;
    if(stepsF > stepsB){
      diff = stepsF - stepsB;
      dir = true;
    }
    else if(stepsB > stepsF){
      diff = stepsB - stepsF;
      dir = false;
    }
    
    //stroke(0);
    //point(posX, posY);
    //noStroke();
    
    deltaX = diff * ((sin(radians(180-orientation))));
    deltaY = diff * ((cos(radians(180-orientation))));
     
    if(dir){
      //stroke(0,0,255);
      //strokeWeight(2);
      //line(40, 40,50, 50);
      //path[points] = posX;
      //points++;
      //path[points] = posY;
      //points++;
      posX+=deltaX;
      posY+=deltaY;
      //path[points] = posX;
      //points++;
      //path[points] = posY;
      //points++;
    }
    else{
      //stroke(255,0,0);
      //strokeWeight(2);
      //line(30,30,40,40);
      //path[points] = posX;
      //points++;
      //path[points] = posY;
      //points++;
      posX-=deltaX;
      posY-=deltaY;
      //path[points] = posX;
      //points++;
      //path[points] = posY;
      //points++;
    }
    
    
      
  }
  
  void rotateRobot(int angle){
    orientation =  angle;
    
  }
  
  void drawRobot(){
    fill(c);
    translate(posX, posY);
    rotate(radians(orientation));
    triangle(0-(width/2),0-(width/2),0+(width/2), 0-(width/2),0,0-(width));
    rect(0-(width/2),0-(width/2), width, height);
  }
  
  float getPosX(){
    return posX;
  }
  
  float getPosY(){
    return posY;
  }
  
  private
  color c;
  float posX;
  float posY;
  float acclX;
  float acclY;
  float acclZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  int width;
  int height;
  float orientation;
}


void serialEvent(Serial port) {
  
  while (port.available() > 0)
  {
    interval =millis();
    //println("ss");
    int c = port.read();
    //println(c);
    if(c == '$')
    {
      started=true;
      serialCount=0;
    }
    if (aligned < 4) 
    {
      // make sure we are properly aligned on a 7-byte packet
      if (serialCount == 0) {
          if (c == '$') aligned++; else aligned = 0;
      } else if (serialCount == 1) {
          if (c == 2) aligned++; else aligned = 0;
      } else if (serialCount == 10) {
          if (c == '\r') aligned++; else aligned = 0;
      } else if (serialCount == 11) {
          if (c == '\n') aligned++; else aligned = 0;
      }
      //println(c + " " + aligned + " " + serialCount);
      serialCount++;
      if (serialCount == 12) serialCount = 0;
    }
    else
    {
      if (serialCount >0 || c =='$')
      {
        packet[serialCount++]= (char)c;
        if (serialCount == 12)
        {
          serialCount=0;
          
          if(packet[3] == 1){
            deltaYaw = -1 * ((int) packet[2]);
          }
          else{
            deltaYaw = (int)packet[2];
          }
          deltaX = (int)packet[4];
          deltaY = (int)packet[5];
          stepsF = (int)packet[6];
          stepsB = (int)packet[7];
          isMine = (int)packet[8];
          aRequest = (int)packet[9];
          
          
          println("Packet Received :");
          //for (int i=0; i<7; i++)
          //{
          //  print("in");
          //  int temp = (int)packet[i];
          //  println(temp);
          //}
          //delay(1000);
          print("M : ");
          println(aRequest);
          //print(" ,X : ");
          //print(deltaX);
          //print(" ,Y : ");
          //println(deltaX);
          
        }
      }
    }
    
  }
}
