#ifndef _UDP_HPP_
#define _UDP_HPP_

namespace CSIR{
namespace UDP{

/**
 * @brief create and bind a upd socket for all ip address
 * 
 * @param port the binded port
 * @param len the return length of a addr_structure 
 * @return the fd of binded socket
 */
int create_and_bind(int port, int& len);

};
};

#endif