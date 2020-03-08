#include "authorized_check.h"

void AuthorizedCheck::split(std::string strtem, char a, std::vector<std::string>& output_vec) {
	std::string::size_type pos1, pos2;
	pos2 = strtem.find(a);
	pos1 = 0;
	while (std::string::npos != pos2)
	{
        output_vec.push_back(strtem.substr(pos1, pos2 - pos1));
        pos1 = pos2 + 1;
        pos2 = strtem.find(a, pos1);
	}
	output_vec.push_back(strtem.substr(pos1));
	return;
}

int AuthorizedCheck::init(const std::string& ip_str, int32_t port) {
    _socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    _address.sin_family = AF_INET;
    _address.sin_port = htons(port); // default : 8080
    //_server_ptr = gethostbyname("172.17.170.205"); // default : "172.17.170.205"
    _server_ptr = gethostbyname(ip_str.c_str()); // "172.17.170.205"
    memcpy((char*) &_address.sin_addr, (char*) _server_ptr->h_addr, _server_ptr->h_length);
    if (connect(_socket_fd, (struct sockaddr*) &_address, sizeof(_address)) == -1) {
        std::cout << "connection error!!" << std::endl;
        return -1;
    }
    return 0;
}

void AuthorizedCheck::gen_request(const std::string& content) {
    _request.clear();
    _request << "POST " << "/TestServer" << " HTTP/1.0\r\n";
    _request << "Host: "<< _ip_str << ":" << std::to_string(_port) << "\r\n";
    _request << "User-Agent: PostmanRuntime/7.15.0\r\n";
    _request << "Content-Type:text/plain\r\n";
    _request << "Content-Length:" << content.size() <<"\r\n";
    _request << "Connection:close\r\n\r\n";
    _request << content.c_str();
}

int AuthorizedCheck::run_check() {
    int result = 0;
    // #step1: send_request
    send_request();
    // #step2: trans_response
    trans_response();
    // #step3: do_check
    result = do_check();

    return result;
}

void AuthorizedCheck::send_request() {
    std::string req_str = _request.str();
    write(_socket_fd, req_str.c_str(), req_str.size());
}

void AuthorizedCheck::trans_response() {
    int offset = 0;
    int rc;
    while(rc = read(_socket_fd, _buffer + offset, 1024)) {
        offset += rc;
    }
    _buffer[offset] = 0;
    //std::cout << "print response: " << _buffer << std::endl;
}

int AuthorizedCheck::do_check() {
    std::string resp(_buffer);
    std::vector<std::string> resp_vec;
    split(resp, '\n', resp_vec);
    int32_t res = 0;
    if (resp_vec.size() != 0) {
        try {
            res = (int32_t)std::stoul(resp_vec[resp_vec.size() - 1]);
        } catch (...) {
        }
    }
    return res;
}
