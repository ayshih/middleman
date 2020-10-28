/*

Tell the interface computer the IP of the flight computer

*/

#define SYS_ID_MM 0xA0
#define KEY_DESTINATION_IP 0xA1
#define PORT_CMD 50501

#include <iostream>
#include <stdint.h>
#include <stdlib.h>

#include "UDPSender.hpp"
#include "Command.hpp"

int main(int argc, char *argv[])
{
    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <interface computer IP> <flight computer IP>\n";
        return -1;
    }

    CommandSender cmdSender(argv[1], PORT_CMD);

    CommandPacket cp(SYS_ID_MM, KEY_DESTINATION_IP, 0);

    int a, b, c, d;
    sscanf(argv[2], "%d.%d.%d.%d", &a, &b, &c, &d);

    cp << (uint8_t)a << (uint8_t)b << (uint8_t)c << (uint8_t)d;

    std::cout << cp << std::endl;
    std::cout << "Packet size: " << cp.getLength() << std::endl;

    cmdSender.send( &cp );

    return 0;
}

