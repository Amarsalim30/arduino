import processing.serial.*; // kütüphane entegresi
import java.awt.event.KeyEvent; 
import java.io.IOException;

Serial myPort; 

String angle="";
String distance="";
String data="";
String noObject;
float pixsDistance;
int iAngle, iDistance;
int index1=0;
int index2=0;
PFont orcFont;

void setup() {
  
 size (1366, 710);
 smooth();
 myPort = new Serial(this,"/dev/ttyUSB0", 9600); // Com portunu seçin
 myPort.bufferUntil('\r'); 

}

void draw() {
  
  fill(98,245,31);

  noStroke();
  fill(0,3); 
  rect(0, 0, width, 1010); 
  
  fill(98,245,31); // yeşil renk

  drawRadar(); 
  drawLine();
  drawObject();
  drawText();
}

void serialEvent (Serial myPort) { 

  data = myPort.readStringUntil('\r');
  data = data.substring(0,data.length()-1);
  
  index1 = data.indexOf(","); 
  angle= data.substring(0, index1); 
  distance= data.substring(index1+1, data.length()); 
  

  iAngle = int(angle);
  iDistance = int(distance);
}

void drawRadar() {
  pushMatrix();
  translate(683,700); 
  noFill();
  strokeWeight(2);
  stroke(98,245,31);
  // draws the arc lines
  arc(0,0,1250,1250,PI,TWO_PI);
  arc(0,0,1125,1125,PI,TWO_PI);
  arc(0,0,1000,1000,PI,TWO_PI);
  arc(0,0,875,875,PI,TWO_PI);
  arc(0,0,750,750,PI,TWO_PI);
  arc(0,0,625,625,PI,TWO_PI);
  arc(0,0,500,500,PI,TWO_PI);
  arc(0,0,375,375,PI,TWO_PI);
  arc(0,0,250,250,PI,TWO_PI);
  arc(0,0,125,125,PI,TWO_PI);
  // draws the angle lines
  line(-700,0,700,0);
  line(0,0,-700*cos(radians(30)),-700*sin(radians(30)));
  line(0,0,-700*cos(radians(60)),-700*sin(radians(60)));
  line(0,0,-700*cos(radians(90)),-700*sin(radians(90)));
  line(0,0,-700*cos(radians(120)),-700*sin(radians(120)));
  line(0,0,-700*cos(radians(150)),-700*sin(radians(150)));
  line(-700*cos(radians(30)),0,700,0);
  popMatrix();
}

void drawObject() {
  pushMatrix();
  translate(683,700); 
  strokeWeight(9);
  stroke(255,10,10); // kırmızı renk
  pixsDistance = iDistance*6.25; 
  // 100 cm ye kadar ölçer
  if(iDistance<100){
    line(pixsDistance*cos(radians(iAngle)),-pixsDistance*sin(radians(iAngle)),700*cos(radians(iAngle)),-700*sin(radians(iAngle)));
  }
  popMatrix();
}

void drawLine() {
  pushMatrix();
  strokeWeight(9);
  stroke(30,250,60);
  translate(683,700); 
  line(0,0,700*cos(radians(iAngle)),-700*sin(radians(iAngle))); 
  popMatrix();
}

void drawText() { 
  
  pushMatrix();
  if(iDistance>100) {
  noObject = "Out of Range";
  }
  else {
  noObject = "In Range";
  }
  fill(0,0,0);
  noStroke();
  rect(0, 1010, width, 1080);
  fill(255,255,0);
  textSize(15);
  text("10cm",750,690);
  text("20cm",815,690);
  text("30cm",875,690);
  text("40cm",940,690);
  text("50cm",1000,690);
  text("60cm",1063,690);
  text("70cm",1125,690);
  text("80cm",1187,690);
  text("90cm",1250,690);
  text("100cm",1310,690);
  
  textSize(35);
  text("Object: " + noObject, 30, 40);
  text("Angle: " + iAngle +" °", 1100, 80);
  text("Distance: "+ iDistance +" cm", 1050, 40);
  
  textSize(25);
  fill(98,245,60);
  translate(390+960*cos(radians(30)),780-960*sin(radians(30)));
  rotate(-radians(-60));
  text("30°",0,0);
  resetMatrix();
  translate(490+960*cos(radians(60)),920-960*sin(radians(60)));
  rotate(-radians(-30));
  text("60°",0,0);
  resetMatrix();
  translate(630+960*cos(radians(90)),990-960*sin(radians(90)));
  rotate(radians(0));
  text("90°",0,0);
  resetMatrix();
  translate(760+960*cos(radians(120)),1000-960*sin(radians(120)));
  rotate(radians(-38));
  text("120°",0,0);
  resetMatrix();
  translate(840+900*cos(radians(150)),920-960*sin(radians(150)));
  rotate(radians(-60));
  text("150°",0,0);
  popMatrix(); 
}
