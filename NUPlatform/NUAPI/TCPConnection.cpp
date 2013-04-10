#include "TCPConnection.h"

TCPConnection::TCPConnection(boost::asio::io_service& io_service) : socket_(io_service), done(true) {
}

TCPConnection::~TCPConnection() {
}

TCPConnection::pointer TCPConnection::create(boost::asio::io_service& io_service) {
    return pointer(new TCPConnection(io_service));
}

tcp::socket& TCPConnection::socket() {
    return socket_;
}

void TCPConnection::start() {
    /* nothing */
    socket_.set_option(tcp::no_delay(true));
}

void TCPConnection::send(std::string data) {
    if (done) {
        boost::asio::async_write(
                socket_,
                boost::asio::buffer(data),
                boost::bind(&TCPConnection::handle_write, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred)
                );
        done = false;
    }
}

void TCPConnection::handle_write(const boost::system::error_code&, size_t) {
    done = true;
}