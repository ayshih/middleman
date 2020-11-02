/*

Tell the interface computer to shut down

*/

#define SYS_ID_MM 0xA0
#define KEY_SHUTDOWN 0xFF
#define PORT_CMD 50501

#include <iostream>
#include <stdint.h>
#include <stdlib.h>

#include "UDPSender.hpp"
#include "Command.hpp"

int main(int argc, char *argv[])
{
    if(argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <interface computer IP>\n";
        return -1;
    }

    CommandSender cmdSender(argv[1], PORT_CMD);

    CommandPacket cp(SYS_ID_MM, KEY_SHUTDOWN, 0);

    std::cout << cp << std::endl;
    std::cout << "Packet size: " << cp.getLength() << std::endl;

    cmdSender.send( &cp );

    return 0;
}

