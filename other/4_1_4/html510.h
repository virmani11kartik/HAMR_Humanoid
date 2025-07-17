/*
 * header for MEAM510 hacks for web interface
 * Nov 2021
 * Use at your own risk
 * 
 */

#ifndef HTML510SERVER_h
#define HTML510SERVER_h

#include <WiFi.h>
#include <WiFiClient.h>
const char HTMLtextType[] PROGMEM = R"===(HTTP/1.1 200 OK
Content-type:text/html

)==="; // note last newline is important  

// cache-control important for streaming images
const char BMPimageType[] PROGMEM = R"===(HTTP/1.1 200 OK
Content-type:image/bmp
Cache-control: no-cache, no-store, must-revalidate

)==="; // note last newline is important  

const char plaintextType[] PROGMEM = R"===(HTTP/1.1 200 OK
Content-type:text/plain

)==="; // note last newline is important  


class HTML510Server
{
private:
  WiFiClient client;
  WiFiServer server; 
  
  #define MAXHANDLERS 30
  int numHandler=0;
  void (*handlerptrs[MAXHANDLERS])();
  String handlerpars[MAXHANDLERS];
  
public:
  HTML510Server(int port): server(port) // default constructor
  {    
  }
  void begin(int port=80);
  void attachHandler(String key, void (*handler)());
  void serve();
  void sendhtml(String data) ;
  void sendplain(String data);
  void sendbmp( unsigned char *bmpHeader, int bmpHeadersize, unsigned char *data, int datasize);
  String getText();
  int getVal();
};

#endif
