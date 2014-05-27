/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#ifndef __UDPCLIENT_H__
#define __UDPCLIENT_H__

#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <sstream>

/**
  @brief Class provinding an UDP client able to communicate over the ethernet with a server identified by its IP address trougth a determined port. To use this class, add the -udp aaa.aaa.aaa.aaa pppp option to the move3d exectutable with a the server ip address and p the port number. After, include the Util-pkg.h. Use then in your functions the global variable "UdpClient * globalUdpClient". To send a message use globalUdpClient->send(std::string("message"));. To receive message use globalUdpClient->receive();.
TODO The receive fonction is a blocant one (i.e. the program wait until there is a data comming from the server). To set it non blocant (i.e. the program do not wait. You have to integrate the reveice method into a loop to check if there is a message or not), use the function globalUdpClient->setblocant(false);.
*/

class UdpClient {
public:
  //Constructors and destructors
  /**
   * @brief Class constructor. Take the server ip address and the port to establish the connection.
   * @param serverIp The server Ip address
   * @param port The port number
   */
  UdpClient(std::string serverIp, int port);
  /**
   * @brief Class destructor. Close the connection with the server.
   */
  virtual ~UdpClient();
  //functions
    //send
  /**
   * @brief Send a text message to the server.
   * @param message The string to send to the server.
   */
  void send(std::string& message);
    //receive
  /**
   * @brief Receive datas from the server. Depend of the usage you have to set this method blocant or not (use the setBlocant method).
   * @return The datas received from the server.
   */
  std::string receive(void);
//////////////  Move3d Function ///////////////
  /**
   * @brief Send a move3d configuration (configPt or double*).
   * @param config The config to send
   * @param size The number of items in the config chart
   */
  void sendConfig(double * config, int size);
//////////////  Move3d Function ///////////////
  //setters and getters
  /** TODO
   * @brief Permit to set the receive function blocant or not.
   * @param block True to set the process blocant, false otherwise
   */
//   void setBlocant(bool block);
  /**
   * @brief Get the server IP address
   * @return The server IP address
   */
  inline std::string& getServerIp(void){return _serverIp;}
  /**
   * @brief Set the server IP address
   * @param serverIp The server IP address
   */
  inline void setServerIp(std::string& serverIp){_serverIp = serverIp;}
  /**
   * @brief Get the communication port
   * @return The communication port
   */
  inline int& getPort(void){ return _port;}
  /**
   * @brief Get the communication port
   * @param port The communication port
   */
  inline void setPort(int& port){_port = port;}
protected:
  //setters and getters
  /**
   * @brief Get the socket
   * @return The socket
   */
  inline int& getSocket(void){return _socket;}
  /**
   * @brief Set the socket
   * @param socket The socket
   */
  inline void setSocket(int& socket){_socket = socket;}
  /**
   * @brief Get the socket address
   * @return The socket address
   */
  inline struct sockaddr_in& getSocketAddress(void){return _si_other;}
  /**
   * @brief Set the socket address
   * @param si_other The socket address
   */
  inline void setSocketAddress(struct sockaddr_in& si_other){_si_other = si_other;}
private:
  /**
   * @brief The server IP address
   */
  std::string _serverIp;
  /**
   * @brief The comminication port
   */
  int _port;
  /**
   * @brief The socket
   */
  int _socket;
  /**
   * @brief The socket address
   */
  struct sockaddr_in _si_other;
//static members
public:
  /**
   * @brief Convert an int to a std::string
   * @param i The int to convert
   * @return The generated std::string
   */
  static std::string convertToString(int i){
    std::string s;
    std::stringstream out;
    out << i;
    s = out.str();
    return s;
  }
  /**
   * @brief Convert a double to a std::string
   * @param d The double to convert
   * @return The generated std::string
   */
  static std::string convertToString(double d){
    std::string s;
    std::stringstream out;
    out << d;
    s = out.str();
    return s;
  }
};

extern UdpClient* globalUdpClient;

#endif
