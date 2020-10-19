/*

Quick commander

*/

#define IP_LOOPBACK "127.0.0.1"
#define PORT_CMD 50501

#include <iostream>
#include <stdint.h>
#include <stdlib.h>

#include "UDPSender.hpp"
#include "Command.hpp"

int main(int argc, char *argv[])
{
    if(argc < 3) {
        std::cerr << "Usage: " << argv[0] << " SystemID CmdType [payload byte] [payload byte] ...\n";
        return -1;
    }

    CommandSender cmdSender(IP_LOOPBACK, PORT_CMD);

    CommandPacket cp(strtoul(argv[1], NULL, 0), strtoul(argv[2], NULL, 0), 0);

    for(int i = 3; i < argc; i++) {
        cp << (uint8_t)strtoul(argv[i], NULL, 0);
    }

    std::cout << cp << std::endl;
    std::cout << "Packet size: " << cp.getLength() << std::endl;

    cmdSender.send( &cp );

    return 0;
}

