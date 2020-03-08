
#include <sys/socket.h>
#include <iostream>
#include <string>
#include <netdb.h>
#include <cstring>
#include <unistd.h>
#include <sstream>
#include <vector>

class AuthorizedCheck {
public:
    AuthorizedCheck() {}
    ~AuthorizedCheck() {
        close(_socket_fd); 
    }

    int init(const std::string& ip_str, int32_t port);
    void gen_request(const std::string& content);
    int run_check();

private:
    void send_request();
    void trans_response();
    int do_check();

    void split(std::string strtem, char a, std::vector<std::string>& output_vec);

    std::string _ip_str;
    int32_t _port = 0;
    int32_t _socket_fd = 0;
    struct sockaddr_in _address;
    struct hostent* _server_ptr;
    std::stringstream _request;
    char _buffer[1024 * 1024] = {0};
};
